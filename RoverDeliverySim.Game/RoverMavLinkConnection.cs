using Asv.IO;
using Asv.Mavlink;
using DotNext;
using R3;

using Stride.Engine;
using System.Threading;
using System.Threading.Tasks;

namespace RoverDeliverySim;

public class RoverMavLinkConnection : AsyncScript
{
    public int Port { get; set; } = 14550;
    public string Host { get; set; } = "127.0.0.1";

    private TaskCompletionSource _tcs;
    private CancellationTokenSource _cts;

    public override async Task Execute()
    {
        _tcs = new TaskCompletionSource();
        _cts = new CancellationTokenSource();
        await using var s = _cts.Token.Register(() => _tcs.TrySetCanceled());

        Log.Info($"[MAVLink] Starting server on {Host}:{Port}");

        var messageFactory = MavlinkV2Protocol.CreateMessageFactory();
        var protocol = Protocol.Create(builder =>
        {
            builder.RegisterMavlinkV2Protocol(messageFactory);
            builder.Features.RegisterBroadcastFeature<MavlinkMessage>();
            builder.Formatters.RegisterSimpleFormatter();
        });

        await using var router = protocol.CreateRouter("ROUTER");
        router.AddUdpPort(p => 
        {
            p.Host = Host;
            p.Port = Port;
        });

        using var subscription = router.OnRxMessage.Subscribe(msg =>
        {
            Log.Info($"[MAVLink] Received message: {msg.Name}");
            // Handle specific payload types here:
            // if (msg is CommandLongPacket cmdLong) { ... }
        });

        while (Game.IsRunning && !_cts.IsCancellationRequested)
        {
            // Update physics or process queued state here
            await Script.NextFrame();
        }

        _cts.Cancel();
    }
}
