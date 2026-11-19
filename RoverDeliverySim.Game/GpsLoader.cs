using Stride.Core.Mathematics;
using Stride.Engine;
using Stride.Input;
using Stride.Rendering.Lights;
using System;
using System.Collections.Generic;
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
    public Entity BuildingPlaceholerTemplate;
    public Entity Camera;

    private void LookAt(Vector2 xz)
    {
        var height = Camera.Transform.Position.Y * 5;
        Camera.Transform.Position = new Vector3(xz.X, height, xz.Y); // Vector2.Y is your world Z
        Camera.Transform.Rotation = Quaternion.RotationX(-MathF.PI / 2);
    }
    private async Task LoadBuildings()
    {
        var data = await File.ReadAllBytesAsync("Data/muchobor.json");
        using var doc = JsonDocument.Parse(data);

        //const int MaxBuildings = 20000;
        var count = 0;
        foreach (var el in doc.RootElement.GetProperty("elements").EnumerateArray())
        {
            if (el.GetProperty("type").GetString() != "way") continue;
            if (!el.TryGetProperty("tags", out var tags) || !tags.TryGetProperty("building", out _)) continue;
            //if (!tags.TryGetProperty("addr:housenumber", out var number) ||
            //    !tags.TryGetProperty("addr:street", out var street)) continue;
            if (!el.TryGetProperty("geometry", out var geometry)) continue;

            var corners = geometry.EnumerateArray()
                .Select(p => (Lat: p.GetProperty("lat").GetDouble(), Lon: p.GetProperty("lon").GetDouble()))
                .ToList();

            // Closed ways repeat the first node at the end.
            if (corners.Count > 1 && corners[0] == corners[^1])
                corners.RemoveAt(corners.Count - 1);
            if (corners.Count < 3) continue;

            var centerLat = corners.Average(c => c.Lat);
            var centerLon = corners.Average(c => c.Lon);

            var (x, z) = PositionUtils.StrideFromGeoJson(centerLat, centerLon);

            var building = BuildingPlaceholerTemplate.Clone();
            building.Transform.Position.X = x;
            building.Transform.Position.Z = z;
            Entity.Scene.Entities.Add(building);
            //LookAt(building.Transform.Position.XZ());
            //Log.Info($"Loaded {street.GetString()} {number.GetString()} at ({x}, {z});");
            count++;
            //if (count >= MaxBuildings) break;
        }

        Log.Info($"Gps data loaded: {count} buildings");
    }

    public override async Task Execute()
    {
        await LoadBuildings();

        while(Game.IsRunning)
        {
            // Do stuff every new frame
            await Script.NextFrame();
        }
    }
}
