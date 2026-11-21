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
    private readonly float BuildingsYaw = MathUtil.DegreesToRadians(180);
    public Entity BuildingPlaceholerTemplate;
    public Entity RoadPlaceholderTemplate;
    public Entity Neighborhood;
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
        var elements = await LoadGeoJson("Data/muchobor.map.json");
        var roads = LoadRoads(elements);
        LoadBuildings(elements, roads);

        while (Game.IsRunning)
        {
            if (Input.IsKeyDown(Keys.F3))
            {
                var pos = Camera.Transform.Position;
                var rot = Camera.Transform.Rotation;
                Log.Warning($"Camera position: {pos.X}:{pos.Y}:{pos.Z}");
                Log.Warning($"Camera rotation: {rot.X}:{rot.Y}:{rot.Z}");
            }
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
        var roadsRoot = Neighborhood.GetChildren().First(e => e.Name == "Roads");
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

    private void LoadBuildings(List<GeoElement> elements, List<RoadSegment> roads)
    {
        var maxBuildings = 10;
        var maxDistance = MaxRoadDistance * Scale;
        var setback = RoadSetback * Scale;
        var modelBB = RoadPlaceholderTemplate.Get<ModelComponent>().Model.BoundingBox;
        var buildingSize = modelBB.Maximum - modelBB.Minimum;
        var buildingsRoot = Neighborhood.GetChildren().First(e => e.Name == "Buildings");

        var buildings = elements
            .Where(e => e.Tags.ContainsKey("building") && e.Tags.ContainsKey("addr:housenumber"))
            .ToList();
        foreach (var building in buildings)
        {
            // Closed ways repeat the first node at the end.
            var corners = building.Geometry;
            if (corners.Last() == corners.First())
            {
                corners.RemoveAt(corners.Count - 1);
            }
            if (corners.Count < 3) continue;

            var points = corners.Select(p => ToScene(p.Lat, p.Lon));
            Vector2 center = points.Aggregate((a, b) => a + b) / points.Count();

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

            var entity = BuildingPlaceholerTemplate.Clone();
            entity.Transform.Position.X = center.X;
            entity.Transform.Position.Z = center.Y;
            entity.Transform.Rotation = Quaternion.RotationY(yaw);

            // NB: only use these for scaling, never use them for coordinates,
            // since we're moving our buildings towards the road.
            //var min = new Vector2(points.Min(p => p.X), points.Min(p => p.Y));
            //var max = new Vector2(points.Max(p => p.X), points.Max(p => p.Y));
            //building.Transform.Scale.X = max.X - min.X;
            //building.Transform.Scale.Z = max.Y - min.Y;

            //Entity.Scene.Entities.Add(building);
            buildingsRoot.AddChild(entity);
            if (buildingsRoot.Count() >= maxBuildings)
            {
                Log.Warning($"Too many buildings, skipped last {buildings.Count - buildingsRoot.Count()}");
            }
        }
        Log.Info($"Loaded {buildingsRoot.Count()} buildings");
    }


}
