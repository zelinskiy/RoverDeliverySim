using Asv.Common;
using Asv.IO;
using Asv.Mavlink;
using Asv.Mavlink.Common;
using Asv.Mavlink.Minimal;
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
            if (positionTimer >= 2f)
            {
                positionTimer = 0f;
                await SendLocalPosition();
                await SendGlobalPosition();
            }
            await Script.NextFrame();
        }

        _cts.Cancel();
        //_router.Dispose(); TBD ???
    }

    private async Task SendLocalPosition()
    {
        var packet = new LocalPositionNedPacket()
        {
            SystemId = _mavlinkId.SystemId,
            ComponentId = _mavlinkId.ComponentId,
            Sequence = _serverDevice.Context.Sequence.GetNextSequenceNumber(),
        };

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
        var packet = new GlobalPositionIntPacket()
        {
            SystemId = _mavlinkId.SystemId,
            ComponentId = _mavlinkId.ComponentId,
            Sequence = _serverDevice.Context.Sequence.GetNextSequenceNumber(),
        };

        // TODO: This should set seq, etc.? But not doing it. What's the point then?
        //var packet = _serverDevice.Context.MessageFactory.Create(GlobalPositionIntPacket.MessageId);

        var pos = Entity.Transform.Position;
        var payload = (GlobalPositionIntPayload)packet.GetPayload();
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
