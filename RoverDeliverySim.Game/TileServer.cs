using Stride.Core.Mathematics;
using Stride.Engine;
using Stride.Input;
using System;
using System.IO;
using System.Net;
using System.Threading.Tasks;

namespace RoverDeliverySim;

public class TileServer : AsyncScript{    
    private HttpListener _listener;
    // TODO: pass proper cancellation token;
    private bool _isRunning;
    private string _url = "http://localhost:8080/";
    public void StartServer()
    {
        _listener = new HttpListener();
        _listener.Prefixes.Add(_url);
        _listener.Start();
        _isRunning = true;
        // Got second thoughts on this...
        Task.Run(ListenLoop);
    }

    private async Task ListenLoop()
    {
        while (_isRunning)
        {
            try
            {
                var context = await _listener.GetContextAsync();
                var request = context.Request;
                var response = context.Response;

                byte[] buffer;

                // Check if QGC is requesting the map tile
                if (request.Url.AbsolutePath.Equals("/map.png", StringComparison.OrdinalIgnoreCase))
                {
                    
                    // Parse X, Y, Z from query string (e.g., /map.png?X=573915&Y=350537&Z=20)
                    int x = int.TryParse(request.QueryString["X"], out var px) ? px : 0;
                    int y = int.TryParse(request.QueryString["Y"], out var py) ? py : 0;
                    int z = int.TryParse(request.QueryString["Z"], out var pz) ? pz : 0;

                    Log.Info($"Requested tile {px}:{py}:{pz}");

                    string tilePath = $"wwwroot/tiles/{z}_{x}_{y}.png";

                    if (File.Exists(tilePath))
                    {
                        buffer = File.ReadAllBytes(tilePath);
                    }
                    else
                    {
                        var mapViewCatch = Entity.Get<SaveMapImageScript>();
                        mapViewCatch.CaptureMapShotAt(x, y, z);
                        if (File.Exists(tilePath))
                        {
                            buffer = File.ReadAllBytes(tilePath);
                        }
                        else
                        {
                            // Fallback to empty tile
                            buffer = File.Exists("wwwroot/empty.png") ? File.ReadAllBytes("wwwroot/empty.png") : Array.Empty<byte>();
                        }                        
                    }

                    response.ContentType = "image/png";
                    response.StatusCode = (int)HttpStatusCode.OK;
                }
                else
                {
                    response.StatusCode = (int)HttpStatusCode.NotFound;
                    buffer = Array.Empty<byte>();
                }

                response.ContentLength64 = buffer.Length;
                await response.OutputStream.WriteAsync(buffer, 0, buffer.Length);
                response.OutputStream.Close();
            }
            catch (HttpListenerException)
            {
                // Thrown when listener stops; safe to ignore
                break;
            }
        }
    }

    public void Stop()
    {
        _isRunning = false;
        _listener?.Stop();
    }


    public override async Task Execute()
    {
        StartServer();
        while (Game.IsRunning)
        {
            // Do stuff every new frame
            await Script.NextFrame();
        }
    }
}
