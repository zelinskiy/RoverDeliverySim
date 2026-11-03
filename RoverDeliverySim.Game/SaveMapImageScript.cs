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
    public string FileName = "map.png";

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
                CaptureOrthoShot();
            }                
        }
    }

    private void CaptureOrthoShot()
    {
        var commandList = Game.GraphicsContext.CommandList;
        
        try
        {
            using var image = RenderTargetAsset.GetDataAsImage(commandList);

            string filePath = Path.Combine(AppContext.BaseDirectory, "local", FileName);
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