using Asv.Common;
using Asv.IO;
using Asv.Mavlink;
using Asv.Mavlink.Common;
using Asv.Mavlink.Minimal;
using BulletSharp.SoftBody;
using DotNext;
using DotNext.Threading;
using Newtonsoft.Json.Linq;
using R3;
using Stride.Core.Mathematics;
using Stride.Engine;
using Stride.Physics;
using System;
using System.Threading;
using System.Threading.Tasks;
using System.Xml.Linq;
using static Stride.Graphics.Buffer;

namespace RoverDeliverySim;

public class RoverMavLinkConnection : AsyncScript
{
    // mavproxy --master=udpin:127.0.0.1:14550 --out=udpout:192.168.1.50:14560 [--no-console]
    public int RemotePort = 14550;
    public string RemoteHost = "127.0.0.1";
    public int LocalPort = 14560;
    public string LocalHost = "127.0.0.1";
    private IServerDevice _serverDevice;
    private CancellationTokenSource _cts;
    private IProtocolRouter _router;
    private MavlinkIdentity _mavlinkId = new MavlinkIdentity(1, 1);

    private static readonly (string, float, MavParamType)[] MAVLINK_PARAMETERS = 
    [
        ("SYSID_THISMAV", 1f,    MavParamType.MavParamTypeUint8),
        ("FRAME_CLASS",   1f,    MavParamType.MavParamTypeUint8),
        ("ARMING_CHECK",  1f,    MavParamType.MavParamTypeInt32),
        ("CRUISE_SPEED",  2f,    MavParamType.MavParamTypeReal32),
        ("WP_RADIUS",     2f,    MavParamType.MavParamTypeReal32),
    ];

    public override async Task Execute()
    {
        var tcs = new TaskCompletionSource();
        _cts = new CancellationTokenSource();
        await using var s = _cts.Token.Register(() => tcs.TrySetCanceled());

        Log.Info($"[MAVLink] Listening on {LocalHost}:{LocalPort} (Rover)");
        Log.Info($"[MAVLink] Sending to {RemoteHost}:{RemotePort} (GCS)");

        var messageFactory = MavlinkV2Protocol.CreateMessageFactory();
        var protocol = Protocol.Create(builder =>
        {
            builder.SetTimeProvider(TimeProvider.System);
            builder.RegisterMavlinkV2Protocol(messageFactory);
            builder.Features.RegisterBroadcastFeature<MavlinkMessage>();
            builder.Formatters.RegisterSimpleFormatter();
        });        

        _router = protocol.CreateRouter("ROUTER");
        var port = _router.AddUdpPort(p => 
        {
            p.Query.Set("rhost", RemoteHost);
            p.Query.Set("rport", RemotePort.ToString());
            p.Host = LocalHost;
            p.Port = LocalPort;
        });
        using var epAddedSubscription = port.EndpointAdded.Subscribe(ep =>
        {
            Log.Info($"[MAVLink] Endpoint added: {ep.DisplayName}");
        });
        using var epRemovedSubscription = port.EndpointRemoved.Subscribe(ep =>
        {
            Log.Info($"[MAVLink] Endpoint removed: {ep.DisplayName}");
        });


        using var rxSub = _router.OnRxMessage.Subscribe(msg =>
        {
            if (msg is MavlinkV2Message mavlinkMsg && mavlinkMsg.FullId != _mavlinkId)
            {
                Log.Info($"[MAVLink] Received {mavlinkMsg.Name} from ({mavlinkMsg.SystemId}, {mavlinkMsg.ComponentId})");
            }
        });

        using var txSub = _router.OnTxMessage.Subscribe(msg =>
        {
            if (msg is MavlinkV2Message mavlinkMsg && mavlinkMsg.FullId == _mavlinkId)
            {
                Log.Info($"[MAVLink] Transmitted {msg.Name}");
            }
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
        
        _serverDevice = ServerDevice.Create(
            _mavlinkId,
            core,
            builder =>
            {
                builder.RegisterHeartbeat();
                builder.RegisterMission();
                builder.RegisterParams();
                builder.RegisterStatus();
                builder.RegisterCommand();
                builder.RegisterCommandIntEx();
                builder.RegisterCommandLongEx();
            });


        // TODO: Use IParamsServerEx
        var paramsService = _serverDevice.GetParams();
        using var readSubscription = paramsService.OnParamRequestRead.Subscribe(request =>
        {
            var name = MavlinkTypesHelper.GetString(request.Payload.ParamId);
            Log.Info($"Parameter read requested: {name}");
        });
        using var listSubscription = paramsService.OnParamRequestList.Subscribe(async req =>
        {
            Log.Info("Parameter list requested");
            for (ushort i = 0; i<MAVLINK_PARAMETERS.Length; i++)
            {                
                await paramsService.SendParamValue(p =>
                {
                    var (name, value, type) = MAVLINK_PARAMETERS[i];
                    MavlinkTypesHelper.SetString(p.ParamId, name);
                    p.ParamValue = value;
                    p.ParamType = type;
                    p.ParamCount = (ushort)MAVLINK_PARAMETERS.Length;
                    p.ParamIndex = i;
                    Log.Info($"Sending {name}={value}");
                });
                await Task.Delay(10);
            }
            
        });
        using var setSubscription = paramsService.OnParamSet.Subscribe(packet =>
        {
            var name = MavlinkTypesHelper.GetString(packet.Payload.ParamId);
            Log.Info($"Parameter set: {name} = {packet.Payload.ParamValue}");
        });
        await paramsService.Init(_cts.Token);
        paramsService.Start();

        var statusService = _serverDevice.GetStatus();
        await statusService.Init(_cts.Token);
        statusService.Info("Rover is OK");
        statusService.Start();

        var heartbeatService = _serverDevice.GetHeartbeat();
        heartbeatService.Set(hb =>
        {
            hb.Type = MavType.MavTypeGroundRover;
            hb.Autopilot = MavAutopilot.MavAutopilotGeneric;
            hb.BaseMode = MavModeFlag.MavModeFlagCustomModeEnabled;
            hb.CustomMode = 0;
            hb.SystemStatus = MavState.MavStateActive;
            hb.MavlinkVersion = 3;
        });
        await heartbeatService.Init(_cts.Token);
        heartbeatService.Start();


        var commandLongService = _serverDevice.GetCommandLongEx();        
        commandLongService[MavCmd.MavCmdGetHomePosition] = (id, packet, ct) =>
        {
            Log.Info("[MAVLink] Get home called");
            return CommandResult.AcceptedTask;
        };
        commandLongService[MavCmd.MavCmdDoSetHome] = (id, packet, ct) =>
        {
            Log.Info("[MAVLink] Set home called");
            return CommandResult.AcceptedTask;
        };
        //commandLongService.

        //var commandIntService = _serverDevice.GetCommandIntEx();
        //commandIntService[MavCmd.MavCmdGetHomePosition] = (id, packet, ct) =>
        //{
        //    Log.Info("[MAVLink] Get home");
        //    return CommandResult.AcceptedTask;
        //};

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
                await SendAttitude();
            }
            await Script.NextFrame();
        }

        _cts.Cancel();
        //_router.Dispose(); TBD ???
    }

    private TPacket createPacket<TPacket, TPayload>(
        Action<TPayload, TransformComponent, RigidbodyComponent> packer)
        where TPacket : MavlinkV2Message<TPayload>, new()
        where TPayload : IPayload
    {
        // TODO: This should set seq, etc.? But not doing it. What's the point then?
        //var packet = _serverDevice.Context.MessageFactory.Create(GlobalPositionIntPacket.MessageId);
        //var payload = (GlobalPositionIntPayload)packet.GetPayload();

        var packet = new TPacket();
        packet.SystemId = _mavlinkId.SystemId;
        packet.ComponentId = _mavlinkId.ComponentId;
        packet.Sequence = _serverDevice.Context.Sequence.GetNextSequenceNumber();

        var t = Entity.Transform;
        var rb = Entity.Get<RigidbodyComponent>();
        var payload = (TPayload)packet.GetPayload();
        packer(payload, t, rb);

        // Hold on with metaprogramming black magic here due to possible performance impact
        //payload.VisitFields((s, ft) => { 
        //    if(ft.Name == "TimeBootMs")
        //    {
        //        var f = s.Peek();
        //        var prop = f.GetType().GetProperty(ft.Name);
        //        prop.SetValue(f, (uint)Game.UpdateTime.Total.TotalMilliseconds);
        //    }
        //});

        return packet;
    }

    private async Task SendLocalPosition()
    {
        var packet = createPacket<LocalPositionNedPacket, LocalPositionNedPayload>((payload, t, rb) =>
        {
            payload.X = t.Position.X;
            payload.Y = t.Position.Y;
            payload.Z = t.Position.Z;
            var v = rb.LinearVelocity;
            payload.Vx = v.Z;
            payload.Vy = v.X;
            payload.Vz = (-1) * v.Y;
            payload.TimeBootMs = (uint)Game.UpdateTime.Total.TotalMilliseconds;
        });
        await _router.Send(packet);
    }

    private async Task SendGlobalPosition()
    {
        var packet = createPacket<GlobalPositionIntPacket, GlobalPositionIntPayload>((payload, t, rb) =>
        {
            var pos = t.Position;
            payload.Lat = 511079000 + (int)(pos.Z * 100);
            payload.Lon = 170385000 + (int)(pos.X * 100);
            payload.Alt = 118000 + (int)(-pos.Y * 1000);
            payload.RelativeAlt = payload.Alt;

            var v = rb.LinearVelocity;
            payload.Vx = (short)(v.Z * 100);
            payload.Vy = (short)(v.X * 100);
            payload.Vz = (short)(-v.Y * 100);

            var forward = t.WorldMatrix.Forward;
            float theta = (float)Math.Atan2(forward.X, -forward.Z);
            float hdg = MathUtil.RadiansToDegrees(theta);
            hdg = (hdg % 360f + 360f) % 360f;
            payload.Hdg = (ushort)Math.Min(hdg * 100f, 35999f);

            payload.TimeBootMs = (uint)Game.UpdateTime.Total.TotalMilliseconds;
        });
        await _router.Send(packet);
    }

    private async Task SendAttitude()
    {
        var packet = createPacket<AttitudePacket, AttitudePayload>((payload, t, rb) => {
            var wm = t.WorldMatrix;
            var f = Vector3.Normalize(wm.Forward);
            var r = Vector3.Normalize(wm.Right);
            var u = Vector3.Normalize(wm.Up);
            payload.Yaw = MathF.Atan2(f.X, f.Z);
            payload.Pitch = MathF.Asin(Math.Clamp(f.Y, -1f, 1f));
            payload.Roll = MathF.Atan2(-r.Y, u.Y);

            var w = rb.AngularVelocity;
            payload.Yawspeed = Vector3.Dot(w, u);
            payload.Pitchspeed = Vector3.Dot(w, r);
            payload.Rollspeed = Vector3.Dot(w, f);

            payload.TimeBootMs = (uint)Game.UpdateTime.Total.TotalMilliseconds;
        });
        await _router.Send(packet);
    }
}
