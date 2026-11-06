using Stride.Core.Mathematics;
using Stride.Engine;
using Stride.Graphics;
using Stride.Input;
using System;
using System.Collections.Concurrent;
using System.IO;
using System.Net.NetworkInformation;
using System.Threading.Tasks;

public class SaveMapImageScript : AsyncScript
{
    private class CaptureRequest
    {
        public int X, Y, Z;
        public TaskCompletionSource<byte[]> Result =
            new(TaskCreationOptions.RunContinuationsAsynchronously);
    }
    private readonly ConcurrentQueue<CaptureRequest> _requestsQueue = new();
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

            //if (Input.IsKeyPressed(Keys.F12))
            //{
            //    string filePath = Path.Combine(AppContext.BaseDirectory, "local", "map.png");
            //    CaptureOrthoShot(filePath);
            //    continue;
            //}

            if (!_requestsQueue.TryDequeue(out var req))
                continue;

            try
            {
                MoveToWebMercatorPos(req.X, req.X, req.Z);
                await Script.NextFrame();
                string filePath = Path.Combine(
                    AppContext.BaseDirectory, 
                    "wwwroot/tiles", 
                    $"{req.Z}_{req.X}_{req.Y}.png"
                );                
                var res = GetTileShot(filePath);
                req.Result.SetResult(res);
            }
            catch (Exception ex)
            {
                // Don't leave the HTTP request hanging
                req.Result.SetException(ex);    
            }
        }
    }
    public Task<byte[]> GetTileShotAt(int x, int y, int z)
    {
        var req = new CaptureRequest { X = x, Y = y, Z = z };
        _requestsQueue.Enqueue(req);
        return req.Result.Task;
    }

    private void MoveToWebMercatorPos(int x, int y, int z)
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
    }

    private byte[] GetTileShot(string tilePath)
    {
        if (File.Exists(tilePath))
        {
            Log.Info($"Read {tilePath}");
            return File.ReadAllBytes(tilePath);
        }
        var commandList = Game.GraphicsContext.CommandList;
        using var image = RenderTargetAsset.GetDataAsImage(commandList);
        using var ms = new MemoryStream();
        image.Save(ms, ImageFileType.Png);
        byte[] png = ms.ToArray();
        File.WriteAllBytes(tilePath, png);
        Log.Info($"Saved {tilePath}");
        return png;
        //return File.ReadAllBytes("wwwroot/empty.png");
    }
}