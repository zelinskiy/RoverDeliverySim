using System;
using System.IO;
using System.Threading.Tasks;
using Stride.Engine;
using Stride.Graphics;
using Stride.Input;
using Stride.Core.Mathematics;

public class SaveMapImageScript : AsyncScript
{
    public Texture RenderTargetAsset;

    public override async Task Execute()
    {
        if (RenderTargetAsset == null)
        {
            Log.Error("RenderTargetAsset is not assigned.");
            return;
        }

        while (Game.IsRunning)
        {
            await Script.NextFrame();

            if (Input.IsKeyPressed(Keys.F12))
            {
                string filePath = Path.Combine(AppContext.BaseDirectory, "local", "map.png");
                CaptureOrthoShot(filePath);
            }                
        }
    }

    public async void CaptureMapShotAt(int x, int y, int z)
    {
        const double earthRadius = 6378137.0;
        const double originShift = Math.PI * earthRadius; // ~20037508.34

        // Total map size in pixels at this zoom level
        double mapSize = 256.0 * Math.Pow(2, z);

        // Meters per pixel resolution at this zoom
        double resolution = (2.0 * originShift) / mapSize;

        // Pixel coordinates of the tile's center
        double pixelX = (x * 256.0) + 128.0;
        double pixelY = (y * 256.0) + 128.0;

        // Convert to Stride world coordinates (XZ plane)
        double worldX = pixelX * resolution - originShift;
        double worldZ = originShift - (pixelY * resolution);

        // Apply to Stride Camera
        var camera = Entity.Get<CameraComponent>();
        Entity.Transform.Position = new Vector3((float)worldX, 500f, (float)worldZ); // 500f is safe height above terrain
        camera.OrthographicSize = (float)(128.0 * resolution); // Half-height of the 256x256 tile

        //await Script.NextFrame();
        string filePath = Path.Combine(AppContext.BaseDirectory, "wwwroot/tiles", $"{z}_{x}_{y}.png");
        CaptureOrthoShot(filePath);
    }

    private void CaptureOrthoShot(string filePath)
    {
        var commandList = Game.GraphicsContext.CommandList;
        
        try
        {
            using var image = RenderTargetAsset.GetDataAsImage(commandList);            
            using var stream = File.Open(filePath, FileMode.Create, FileAccess.Write);
            image.Save(stream, ImageFileType.Png);

            Log.Info($"Saved {filePath}");
        }
        catch (Exception ex)
        {
            Log.Error("Failed to save map image", ex);
        }
    }
}