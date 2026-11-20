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
    [DataMember]
    public (double Lat, double Lon) GpsOrigin { get; } = (51.1041661, 16.9631966);
    public const float Scale = 1f;
    public const float RoadSetback = 4f; // in real meters
    private const float MaxRoadDistance = 40f; // in real meters 
    private readonly float BuildingsYaw = MathUtil.DegreesToRadians(90);
    public Entity BuildingPlaceholerTemplate;
    public Entity RoadPlaceholderTemplate;
    public Entity Camera;

    // Either road or building
    private sealed record GeoElement(
        long Id, 
        Dictionary<string, string> Tags, 
        List<(double Lat, double Lon)> Geometry
    );

    private readonly record struct RoadSegment(
        Vector2 Begin, // XZ
        Vector2 End, // XZ
        float HalfWidth
    );

    // TODO: check this list
    private static readonly HashSet<string> DrivableHighways = new()
    {
        "motorway", 
        "trunk", 
        "primary", 
        "secondary", 
        "tertiary",
        "unclassified", 
        "residential", 
        "living_street", 
        "service",
    };

    public override async Task Execute()
    {
        var elements = await LoadGeoJson("Data/muchobor.json");
        var roads = LoadRoads(elements);
        LoadBuildings(elements, roads);

        while (Game.IsRunning)
        {
            // Do stuff every new frame
            await Script.NextFrame();
        }
    }

    private static async Task<List<GeoElement>> LoadGeoJson(string path)
    {
        var data = await File.ReadAllBytesAsync(path);
        using var doc = JsonDocument.Parse(data);

        var elements = new List<GeoElement>();
        foreach (var el in doc.RootElement.GetProperty("elements").EnumerateArray())
        {
            if (el.GetProperty("type").GetString() != "way") continue;
            if (!el.TryGetProperty("geometry", out var geometry)) continue;

            var tags = el.TryGetProperty("tags", out var t)
                ? t.EnumerateObject().ToDictionary(p => p.Name, p => p.Value.GetString() ?? "")
                : [];

            var points = geometry.EnumerateArray()
                .Where(p => p.ValueKind == JsonValueKind.Object)
                .Select(p => (Lat: p.GetProperty("lat").GetDouble(), Lon: p.GetProperty("lon").GetDouble()))
                .ToList();
            if (points.Count < 2) continue;

            elements.Add(new GeoElement(el.GetProperty("id").GetInt64(), tags, points));
        }

        return elements;
    }

    private Vector2 ToScene(double lat, double lon)
    {
        var (x, z) = PositionUtils.StrideFromGeoJson(lat, lon);
        return new Vector2(x, z) * Scale;
    }

    // Real-world road width in metres: explicit "width" tag, else lanes * 3 m, else 6 m.
    private static float RoadWidthMeters(Dictionary<string, string> tags)
    {
        if (tags.TryGetValue("width", out var w) &&
            float.TryParse(w, NumberStyles.Float, CultureInfo.InvariantCulture, out var width))
            return width;

        if (tags.TryGetValue("lanes", out var l) && int.TryParse(l, out var lanes))
            return lanes * 3f;

        return 6f;
    }

    private List<RoadSegment> LoadRoads(List<GeoElement> elements)
    {
        var modelBB = RoadPlaceholderTemplate.Get<ModelComponent>().Model.BoundingBox;
        var roadSize = modelBB.Maximum - modelBB.Minimum;
        //var roadSize = RoadPlaceholderTemplate.Transform.Scale.XZ();
        var segments = new List<RoadSegment>();

        foreach (var element in elements)
        {
            if (!element.Tags.TryGetValue("highway", out var highway)) continue;
            if (!DrivableHighways.Contains(highway.Replace("_link", ""))) continue;

            element.Tags.TryGetValue("name", out var roadName);

            //var width = RoadWidthMeters(element.Tags) * Scale;
            var pts = element.Geometry.Select(g => ToScene(g.Lat, g.Lon)).ToList();

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
                //piece.Transform.Scale.Z = width / roadSize.Y;                
                Entity.Scene.Entities.Add(piece);

                segments.Add(new RoadSegment(a, b, piece.Transform.Scale.Z / 2));
            }
            Log.Info($"Loaded {roadName ?? "anonymous"}");
        }
        Log.Info($"Loaded {segments.Count} road segments");
        return segments;
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

    private void LookAt(Vector2 xz)
    {
        var height = Camera.Transform.Position.Y * 5;
        Camera.Transform.Position = new Vector3(xz.X, height, xz.Y); // Vector2.Y is your world Z
        Camera.Transform.Rotation = Quaternion.RotationX(-MathF.PI / 2);
    }

    private List<Entity> LoadBuildings(List<GeoElement> elements, List<RoadSegment> roads)
    {
        var MaxBuildings = 1000;
        var maxDistance = MaxRoadDistance * Scale;
        var setback = RoadSetback * Scale;
        List<Entity> placedBuildings = [];

        foreach (var way in elements)
        {
            if (!way.Tags.ContainsKey("building")) continue;

            // Closed ways repeat the first node at the end.
            var corners = way.Geometry;
            var n = corners.Count;
            if (n > 1 && corners[0] == corners[n - 1]) n--;
            if (n < 3) continue;

            var center = Vector2.Zero;
            for (var i = 0; i < n; i++)
                center += ToScene(corners[i].Lat, corners[i].Lon);
            center /= n;

            var yaw = 0f;
            if (TryFindNearestRoad(roads, center, out var road, out var closest, out var distance)
                && distance <= maxDistance && distance > 1e-4f)
            {
                var toRoad = (closest - center) / distance;   // unit vector: building -> road

                // Keep the side of the street and the position along it; only change the distance from the centreline.
                center = closest - toRoad * (road.HalfWidth + setback);

                // Local +Z (plus the model's front offset) points at the road.
                yaw = MathF.Atan2(toRoad.X, toRoad.Y) + BuildingsYaw;
            }

            var building = BuildingPlaceholerTemplate.Clone();
            building.Transform.Position.X = center.X;
            building.Transform.Position.Z = center.Y;
            building.Transform.Rotation = Quaternion.RotationY(yaw);
            Entity.Scene.Entities.Add(building);
            placedBuildings.Add(building);
            if (placedBuildings.Count > MaxBuildings) break;
        }

        Log.Info($"Loaded {placedBuildings.Count} buildings");
        return placedBuildings;
    }


}
