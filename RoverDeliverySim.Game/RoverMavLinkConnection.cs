using Asv.Common;
using Asv.IO;
using Asv.Mavlink;
using Asv.Mavlink.Common;
using Asv.Mavlink.Minimal;
using DotNext.Threading;
using R3;
using Stride.Engine;
using Stride.Physics;
using Stride.Core.Mathematics;
using System;
using System.Threading;
using System.Threading.Tasks;
using DotNext;

namespace RoverDeliverySim;

public class RoverMavLinkConnection : AsyncScript
{
    public int Port { get; set; } = 14550;
    public string Host { get; set; } = "127.0.0.1";
    private IServerDevice _serverDevice;
    private CancellationTokenSource _cts;
    private IProtocolRouter _router;
    private MavlinkIdentity _mavlinkId = new MavlinkIdentity(1, 1);

    public override async Task Execute()
    {
        var tcs = new TaskCompletionSource();
        _cts = new CancellationTokenSource();
        await using var s = _cts.Token.Register(() => tcs.TrySetCanceled());

        Log.Info($"[MAVLink] Starting server on {Host}:{Port}");

        var messageFactory = MavlinkV2Protocol.CreateMessageFactory();
        var protocol = Protocol.Create(builder =>
        {
            builder.SetTimeProvider(TimeProvider.System);
            builder.RegisterMavlinkV2Protocol(messageFactory);
            builder.Features.RegisterBroadcastFeature<MavlinkMessage>();
            builder.Formatters.RegisterSimpleFormatter();
        });        

        _router = protocol.CreateRouter("ROUTER");
        _router.AddUdpPort(p => 
        {
            p.Host = Host;
            p.Port = Port;
        });

        using var rxSub = _router.OnRxMessage.Subscribe(msg =>
        {
            if (msg is MavlinkV2Message mavlinkMsg)
            {
                if(mavlinkMsg.FullId != _mavlinkId)
                {
                    Log.Info($"[MAVLink] Received {mavlinkMsg.Name} from ({mavlinkMsg.SystemId}, {mavlinkMsg.ComponentId})");
                }
            }
        });

        using var txSub = _router.OnTxMessage.Subscribe(msg =>
        {
            Log.Info($"[MAVLink] Transmitted {msg.Name}");
        });

        var clientSeq = new PacketSequenceCalculator();

        var core = new CoreServices(
            _router,
            messageFactory,
            clientSeq,
            logFactory: null,
            timeProvider: TimeProvider.System,
            new DefaultMeterFactory()
        );

        var builder = new ServerDeviceBuilder(_mavlinkId, core);
        builder.RegisterHeartbeat();
        builder.RegisterMission();
        builder.RegisterCommand();
        builder.RegisterParams();
        builder.RegisterStatus();
        _serverDevice = builder.Build();


        var heartbeatService = _serverDevice.GetHeartbeat();
        heartbeatService.Set(hb =>
        {
            hb.Type = MavType.MavTypeGroundRover;
            hb.Autopilot = MavAutopilot.MavAutopilotArdupilotmega;
            hb.BaseMode = MavModeFlag.MavModeFlagCustomModeEnabled;
            hb.CustomMode = 0;
            hb.SystemStatus = MavState.MavStateActive;
            hb.MavlinkVersion = 3;
        });


        float positionTimer = 0f;

        while (Game.IsRunning && !_cts.IsCancellationRequested)
        {
            float deltaTime = (float)Game.UpdateTime.Elapsed.TotalSeconds;            

            positionTimer += deltaTime;
            if (positionTimer >= 0.1f)
            {
                positionTimer = 0f;
                //await SendLocalPosition();
                await SendGlobalPosition();
            }

            // Update physics or process queued state here
            await Script.NextFrame();
        }

        _cts.Cancel();
        //_router.Dispose(); TBD ???
    }

    private async Task SendLocalPosition()
    {
        var packet = new LocalPositionNedPacket();
        //{
        //    SystemId = 1,
        //    ComponentId = 1,
        //};

        var pos = Entity.Transform.Position;
        packet.Payload.X = pos.X;
        packet.Payload.Y = pos.Y;
        packet.Payload.Z = pos.Z;

        var rigidBody = Entity.Get<RigidbodyComponent>();
        var vel = rigidBody.LinearVelocity;
        packet.Payload.Vx = vel.Z;
        packet.Payload.Vy = vel.X;
        packet.Payload.Vz = (-1) * vel.Y;

        await _router.Send(packet);
    }

    private async Task SendGlobalPosition()
    {
        byte seq = _serverDevice.Context.Sequence.GetNextSequenceNumber();
        var packet = new GlobalPositionIntPacket()
        {
            SystemId = 1,
            ComponentId = 1,
            Sequence = seq,
        };

        var pos = Entity.Transform.Position;
        var payload = packet.Payload;
        payload.Lat = 511079000 + (int)(pos.Z * 100);
        payload.Lon = 170385000 + (int)(pos.X * 100);
        payload.Alt = 118000 + (int)(-pos.Y * 1000);
        payload.RelativeAlt = payload.Alt;

        var rigidBody = Entity.Get<RigidbodyComponent>();
        var vel = rigidBody.LinearVelocity;
        payload.Vx = (short)(vel.Z * 100);
        payload.Vy = (short)(vel.X * 100);
        payload.Vz = (short)(-vel.Y * 100);

        var forward = Entity.Transform.WorldMatrix.Forward;
        float theta = (float)Math.Atan2(-forward.X, -forward.Z);
        float hdg = MathUtil.RadiansToDegrees(theta);
        if (hdg < 0f) hdg += 360f;
        payload.Hdg = (ushort)(hdg * 100f); ;

        payload.TimeBootMs = (uint)(Game.UpdateTime.Total.TotalMilliseconds % uint.MaxValue);
        
        await _router.Send(packet);
    }
}
