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
    const double SceneOriginLat = 0;
    const double SceneOriginLon = 0;
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

            if (Input.IsKeyPressed(Keys.F12))
            {
                var timestamp = DateTime.Now.ToString("yyyyMMddHHmmssffff");
                string filePath = Path.Combine(AppContext.BaseDirectory, "local", $"map.{timestamp}.png");
                GetTileShot(filePath);
                continue;
            }

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

    private static (double, double) LatLonToMercator(double lat, double lon)
    {
        const double R = 6378137.0;
        double mx = lon * Math.PI / 180.0 * R;
        double my = Math.Log(Math.Tan(Math.PI / 4.0 + lat * Math.PI / 360.0)) * R;
        return (mx, my);
    }

    private void MoveToWebMercatorPos(int x, int y, int z)
    {
        const double originShift = Math.PI * 6378137.0;
        double mapSize = 256.0 * Math.Pow(2, z);
        double resolution = (2.0 * originShift) / mapSize;

        double tileMx = (x * 256.0 + 128.0) * resolution - originShift;
        double tileMy = (y * 256.0 + 128.0) * resolution - originShift;

        var (originMx, originMy) = LatLonToMercator(SceneOriginLat, SceneOriginLon);
        var (dx, dy) = (tileMx - originMx, tileMy - originMy);
        double k = Math.Cos(SceneOriginLat * Math.PI / 180.0);

        var camera = Entity.Get<CameraComponent>();
        Entity.Transform.Position = new Vector3((float)dx, 500f, (float)(dy));
        camera.OrthographicSize = (float)(256.0 * resolution);
        Log.Info($"Moved map view camera to {dx}, {dy}");
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
        using var fstream = File.Open(tilePath, FileMode.Create);
        image.Save(fstream, ImageFileType.Png);
        using var mstream = new MemoryStream();
        fstream.CopyTo(mstream);        
        Log.Info($"Saved {tilePath}");
        //image.Save(ms, ImageFileType.Png);
        //File.WriteAllBytes(tilePath, png);
        //return png;
        return mstream.ToArray();
        //return File.ReadAllBytes("wwwroot/empty.png");
    }
}