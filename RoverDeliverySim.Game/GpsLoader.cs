using NetTopologySuite.Features;
using NetTopologySuite.Geometries;
using NetTopologySuite.IO.Converters;
using System.Text.Json;
using NtsPoint = NetTopologySuite.Geometries.Point;
using Stride.Core;
using Stride.Core.Mathematics;
using Stride.Engine;
using Stride.Input;
using Stride.Rendering.Lights;
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Runtime.Serialization;
using System.Text;
using System.Text.Json;
using System.Threading.Tasks;


namespace RoverDeliverySim;

public class GpsLoader : AsyncScript
{
    public (double Lat, double Lon) GpsOrigin { get; } = (51.1041661, 16.9631966);
    public const float Scale = .5f;
    public const float RoadSetback = 4f; // in real meters
    private const float MaxRoadDistance = 40f; // in real meters
    public const float RoadWidth = 6f; // in real meters
    private readonly float BuildingsYaw = MathUtil.DegreesToRadians(180);
    public Entity BuildingPlaceholerTemplate;
    public Entity RoadPlaceholderTemplate;
    public Entity Neighborhood;
    public Entity Camera;

    private sealed record Address(string StreetName, string StreetId, string PostalCode, string HouseNumber);

    private abstract record GeoFeature
    {
        public required string Id { get; init; }
        public required Geometry Geometry { get; init; }
    }

    private sealed record BuildingGeoFeature : GeoFeature
    {
        public required IReadOnlyList<Address> Addresses { get; init; }
    }

    private sealed record AddressGeoFeature : GeoFeature
    {
        public required Address Address { get; init; }
    }

    private sealed record StreetGeoFeature : GeoFeature
    {
        public required string StreetName { get; init; }
    }

    private readonly record struct RoadSegment(
        Vector2 Begin, // XZ
        Vector2 End, // XZ
        float HalfWidth
    );

    public override async Task Execute()
    {
        var elements = await LoadGeoJson("Data/muchobor.map.json");
        var roads = LoadRoads(elements);
        LoadBuildings(elements, roads);

        Camera.Transform.Rotation = Quaternion.RotationYawPitchRoll(
            0.06039264f, -0.51602393f, 0.006604771f
        );
        Camera.Transform.Position = new Vector3(200, 150, 375);

        while (Game.IsRunning)
        {
            if (Input.IsKeyDown(Keys.F3))
            {
                Log.Warning($"Camera position: {Camera.Transform.Position}");
                Log.Warning($"Camera rotation: {Camera.Transform.Rotation.YawPitchRoll}");
            }
            await Script.NextFrame();
        }
    }

    private static readonly JsonSerializerOptions JsonOptions = CreateJsonOptions();

    private static JsonSerializerOptions CreateJsonOptions()
    {
        var o = new JsonSerializerOptions { PropertyNamingPolicy = JsonNamingPolicy.SnakeCaseLower };
        o.Converters.Add(new GeoJsonConverterFactory());
        return o;
    }

    private static async Task<List<GeoFeature>> LoadGeoJson(string path)
    {
        await using var stream = File.OpenRead(path);
        var collection = await JsonSerializer.DeserializeAsync<FeatureCollection>(stream, JsonOptions)
            ?? throw new InvalidDataException($"{path} is not a FeatureCollection");
        return collection.Select(ToGeoFeature).ToList();
    }

    // Attribute values come back as boxed objects; round-tripping through JsonElement
    // gives typed access whatever the boxed type is.
    private static T? Attr<T>(IFeature f, string name) =>
        f.Attributes.Exists(name)
            ? JsonSerializer.SerializeToElement(f.Attributes[name], JsonOptions).Deserialize<T>(JsonOptions)
            : default;

    private static GeoFeature ToGeoFeature(IFeature f)
    {
        var kind = Attr<string>(f, "kind");
        return kind switch
        {
            "building" => new BuildingGeoFeature
            {
                Id = Attr<long>(f, "osm_id").ToString(CultureInfo.InvariantCulture),
                Geometry = f.Geometry,
                Addresses = Attr<List<Address>>(f, "addresses") ?? [],
            },
            "street" => new StreetGeoFeature
            {
                Id = Attr<string>(f, "id")!,
                Geometry = f.Geometry,
                StreetName = Attr<string>(f, "name")!,
            },
            "address" => ToAddressFeature(f),
            _ => throw new InvalidDataException($"Unknown feature kind '{kind}'")
        };
    }

    private static AddressGeoFeature ToAddressFeature(IFeature f)
    {
        var address = new Address(
            Attr<string>(f, "street")!,
            Attr<string>(f, "street_id")!,
            Attr<string>(f, "postal_code")!,
            Attr<string>(f, "house_number")!);
        return new AddressGeoFeature
        {
            Id = $"{address.StreetId}:{address.HouseNumber}",
            Geometry = f.Geometry,
            Address = address,
        };
    }

    private Vector2 ToScene(double lat, double lon)
    {
        var (x, z) = PositionUtils.StrideFromGeoJson(lat, lon);
        return new Vector2(x, z) * Scale;
    }

    // Y = lat, X = lon
    private Vector2 ToScene(Coordinate c) => ToScene(c.Y, c.X);

    private static IEnumerable<Coordinate[]> FlattenPolyline(Geometry g) => g switch
    {
        LineString l => [l.Coordinates],
        MultiLineString m => m.Geometries.Select(x => x.Coordinates),
        _ => throw new InvalidDataException($"Expected a line geometry, got {g.GeometryType}")
    };

    private Vector2 Centroid(Polygon footprint)
    {
        // NTS rings are closed (first == last), drop the duplicate.
        var pts = footprint.ExteriorRing.Coordinates[..^1].Select(ToScene).ToList();
        return pts.Aggregate((a, b) => a + b) / pts.Count;
    }

    private static string Label(IEnumerable<Address> addresses) =>
        string.Join(", ", addresses.Select(a => $"{a.StreetName} {a.HouseNumber}"));

    private void LookAt(Vector2 xz)
    {
        var height = Camera.Transform.Position.Y * 5;
        Camera.Transform.Position = new Vector3(xz.X, height, xz.Y); // Vector2.Y is your world Z
        Camera.Transform.Rotation = Quaternion.RotationX(-MathF.PI / 2);
    }

    // Real-world road width in metres: explicit "width" tag, else lanes * 3 m, else 6 m.
    //private static float RoadWidthMeters(Dictionary<string, string> tags)
    //{
    //    if (tags.TryGetValue("width", out var w) &&
    //        float.TryParse(w, NumberStyles.Float, CultureInfo.InvariantCulture, out var width))
    //        return width;

    //    if (tags.TryGetValue("lanes", out var l) && int.TryParse(l, out var lanes))
    //        return lanes * 3f;

    //    return 6f;
    //}

    private List<RoadSegment> LoadRoads(List<GeoFeature> features)
    {
        var modelBB = RoadPlaceholderTemplate.Get<ModelComponent>().Model.BoundingBox;
        var roadSize = modelBB.Maximum - modelBB.Minimum;
        var roadWidth = RoadWidth * Scale;
        var segments = new List<RoadSegment>();

        foreach (var street in features.OfType<StreetGeoFeature>())
            foreach (var line in FlattenPolyline(street.Geometry))
            {
                var pts = line.Select(ToScene).ToList();

                for (var i = 0; i < pts.Count - 1; i++)
                {
                    var a = pts[i];
                    var b = pts[i + 1];
                    var d = b - a;
                    var length = d.Length();
                    if (length < 1e-4f) continue;

                    var piece = RoadPlaceholderTemplate.Clone();
                    piece.Transform.Position.X = (a.X + b.X) / 2;
                    piece.Transform.Position.Z = (a.Y + b.Y) / 2;
                    // Local +X (the stretched axis) must point along the segment.
                    piece.Transform.Rotation = Quaternion.RotationY(MathF.Atan2(-d.Y, d.X));
                    piece.Transform.Scale.X = length / roadSize.X;
                    piece.Transform.Scale.Z = roadWidth / roadSize.Z;
                    Entity.Scene.Entities.Add(piece);

                    segments.Add(new RoadSegment(a, b, roadWidth / 2));
                }
                Log.Info($"Loaded {street.StreetName}");
            }
        Log.Info($"Loaded {segments.Count} road segments");
        return segments;
    }

    // Buildings that have addresses + standalone address points (no building polygon).
    private IEnumerable<(string Label, Vector2 Center)> BuildingSites(List<GeoFeature> features)
    {
        foreach (var f in features)
        {
            switch (f)
            {
                // Skip noname buildings like sheds etc.
                case BuildingGeoFeature { Addresses.Count: > 0 } b:
                    yield return (Label(b.Addresses), Centroid((Polygon)b.Geometry));
                    break;
                case AddressGeoFeature a:
                    yield return (Label([a.Address]), ToScene(a.Geometry.Coordinate));
                    break;
            }
        }
    }

    private static bool TryFindNearestRoad(
        List<RoadSegment> roads,
        Vector2 p,
        out RoadSegment best,
        out Vector2 bestPoint,
        out float bestDistance)
    {
        best = default;
        bestPoint = default;
        bestDistance = float.MaxValue;

        foreach (var r in roads)
        {
            var ab = r.End - r.Begin;
            var t = Vector2.Dot(p - r.Begin, ab)
                / ab.LengthSquared();
            t = Math.Clamp(t, 0f, 1f);
            var q = r.Begin + ab * t;
            var d = Vector2.Distance(p, q);
            if (d < bestDistance)
            {
                bestDistance = d;
                best = r;
                bestPoint = q;
            }
        }

        return bestDistance < float.MaxValue;
    }

    private void LoadBuildings(List<GeoFeature> features, List<RoadSegment> roads)
    {
        var maxBuildings = 10;
        var maxDistance = MaxRoadDistance * Scale;
        var setback = RoadSetback * Scale;
        var buildingsRoot = Neighborhood.GetChildren().First(e => e.Name == "Buildings");

        foreach (var (label, siteCenter) in BuildingSites(features))
        {
            var center = siteCenter;
            var yaw = 0f;
            if (TryFindNearestRoad(roads, center, out var road, out var closest, out var distance)
                && distance <= maxDistance && distance > 1e-4f)
            {
                var toRoad = (closest - center) / distance;   // unit vector: building -> road

                // Keep the side of the street and the position along it;
                // only change the distance from the centreline.
                center = closest - toRoad * (road.HalfWidth + setback);

                // Local +Z (plus the model's front offset) points at the road.
                yaw = MathF.Atan2(toRoad.X, toRoad.Y) + BuildingsYaw;
            }

            var entity = BuildingPlaceholerTemplate.Clone();
            entity.Name = label;
            entity.Transform.Position.X = center.X;
            entity.Transform.Position.Z = center.Y;
            entity.Transform.Rotation = Quaternion.RotationY(yaw);

            // NB: only use these for scaling, never use them for coordinates,
            // since we're moving our buildings towards the road.
            //var min = new Vector2(points.Min(p => p.X), points.Min(p => p.Y));
            //var max = new Vector2(points.Max(p => p.X), points.Max(p => p.Y));
            //building.Transform.Scale.X = max.X - min.X;
            //building.Transform.Scale.Z = max.Y - min.Y;

            buildingsRoot.AddChild(entity);

            if (buildingsRoot.Count() >= maxBuildings)
            {
                Log.Warning($"Reached maxBuildings ({maxBuildings}), stopping");
                break;
            }
        }
        Log.Info($"Loaded {buildingsRoot.Count()} buildings");
    }


}
