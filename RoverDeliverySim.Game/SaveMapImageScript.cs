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
    public static readonly double SceneOriginLat = 51.1079;
    public static readonly double SceneOriginLon = 17.0385;
    public static readonly double SceneOriginAlt = 118.0;
    private readonly ConcurrentQueue<CaptureRequest> _requestsQueue = new();
    public Texture RenderTargetAsset;
    private Vector3 _initPosition;

    public override async Task Execute()
    {
        _initPosition = Entity.Transform.Position;
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
                // Move over rover?
                Entity.Transform.Position = _initPosition;
                await Script.NextFrame();
                var timestamp = DateTime.Now.ToString("yyyyMMddHHmmssffff");
                string filePath = Path.Combine(AppContext.BaseDirectory, "local", $"map.{timestamp}.png");
                GetTileShot(filePath);
                continue;
            }

            if (!_requestsQueue.TryDequeue(out var req))
                continue;

            try
            {
                MoveToWebMercatorPos(req.X, req.Y, req.Z);
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
        double resolution = (2.0 * originShift) / (256.0 * Math.Pow(2, z));

        // Tile centre in Mercator meters (XYZ: Y grows southward)
        double tileMx = (x * 256.0 + 128.0) * resolution - originShift;
        double tileMy = originShift - (y * 256.0 + 128.0) * resolution; // northing

        var (originMx, originMy) = LatLonToMercator(SceneOriginLat, SceneOriginLon);
        double k = Math.Cos(SceneOriginLat * Math.PI / 180.0);

        double east = (tileMx - originMx) * k;   // ground meters
        double north = (tileMy - originMy) * k;

        var camera = Entity.Get<CameraComponent>();
        Entity.Transform.Position = new Vector3((float)east, 500f, (float)(-north));
        camera.OrthographicSize = (float)(256.0 * resolution * k);
        Log.Info($"Moved map view camera to E={east}, N={north}");
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