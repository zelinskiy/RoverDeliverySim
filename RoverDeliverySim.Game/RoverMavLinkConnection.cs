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
using System.Collections.Concurrent;
using System.Threading.Tasks;
using System.Xml.Linq;
using static Stride.Graphics.Buffer;
using System.Linq;
using Stride.Core.Diagnostics;
using Asv.Mavlink.Diagnostic;

namespace RoverDeliverySim;

public class RoverMavLinkConnection : AsyncScript
{
    //TODO: 
    // - Handle SYSTEM_TIME
    // mavproxy --master=udpin:127.0.0.1:14550 --out=udpout:192.168.1.50:14560 [--no-console]
    public int RemotePort = 14550;
    public string RemoteHost = "127.0.0.1";
    public int LocalPort = 14560;
    public string LocalHost = "127.0.0.1";
    public readonly ArduCopterMode[] SupportedModes = 
    {
        ArduCopterMode.Stabilize,
        ArduCopterMode.PosHold,
        ArduCopterMode.Guided,
        ArduCopterMode.Auto,
        ArduCopterMode.Rtl,
        ArduCopterMode.SmartRtl,
    };
    private IServerDevice _serverDevice;
    private CancellationTokenSource _cts;
    private MavlinkIdentity _mavlinkId = new MavlinkIdentity(1, 1);
    private readonly string[] MUTED_MESSAGES = 
    { 
        "ATTITUDE", 
        "HEARTBEAT", 
        "GLOBAL_POSITION_INT", 
        "LOCAL_POSITION_INT" 
    }; 

    private static readonly (string, float, MavParamType)[] MAVLINK_PARAMETERS =
    [
        ("SYSID_THISMAV", 1f,    MavParamType.MavParamTypeUint8),
        ("FRAME_CLASS",   1f,    MavParamType.MavParamTypeUint8),
        ("ARMING_CHECK",  1f,    MavParamType.MavParamTypeInt32),
        ("CRUISE_SPEED",  2f,    MavParamType.MavParamTypeReal32),
        ("WP_RADIUS",     2f,    MavParamType.MavParamTypeReal32),

        ("FLTMODE1",      0f,    MavParamType.MavParamTypeUint8),   // Manual
        ("FLTMODE2",      4f,    MavParamType.MavParamTypeUint8),   // Hold
        ("FLTMODE3",      15f,   MavParamType.MavParamTypeUint8),   // Guided
        ("FLTMODE4",      10f,   MavParamType.MavParamTypeUint8),   // Auto
        ("GUID_SPEED",    2f,    MavParamType.MavParamTypeReal32),
        ("GUID_TIMEOUT",  3f,    MavParamType.MavParamTypeReal32),
    ];

    private class RoverModeHandler(Logger log, ICustomMode mode) : IWorkModeHandler
    {
        public ICustomMode Mode { get; } = mode;
        public Logger Log { get; } = log;
        public Task Init(CancellationToken cancel)
        {
            if (Mode is ArduCopterMode mode)
            {
                Log.Info($"Handler for {mode.Name} mode created");
                //if (mode == ArduCopterMode.Guided)
                //{
                //    Log.Info($"Handler for Guided mode created");
                //}
            }
            return Task.CompletedTask;
        }

        public Task Destroy(CancellationToken cancel)
        {
            if (Mode is ArduCopterMode mode)
            {
                Log.Info($"Handler for {mode.Name} mode destroyed");
                //if (mode == ArduCopterMode.Guided)
            }
            return Task.CompletedTask;
        }
        public void Dispose()
        {

        }
    }

    public override async Task Execute()
    {
        var tcs = new TaskCompletionSource();
        _cts = new CancellationTokenSource();
        _cts.Token.Register(() => tcs.TrySetCanceled());        

        var messageFactory = MavlinkV2Protocol.CreateMessageFactory();
        var protocol = Protocol.Create(builder =>
        {
            builder.SetTimeProvider(TimeProvider.System);
            builder.RegisterMavlinkV2Protocol(messageFactory);
            builder.Features.RegisterBroadcastFeature<MavlinkMessage>();
            builder.Formatters.RegisterSimpleFormatter();
        });
        var router = SetupRouter(protocol);
        var core = new CoreServices(
            router,
            messageFactory,
            new PacketSequenceCalculator(),
            logFactory: null,
            timeProvider: TimeProvider.System,
            new DefaultMeterFactory()
        );

        _serverDevice = ServerDevice.Create(
            _mavlinkId,
            core,
            builder =>
            {
                builder.RegisterHeartbeat();
                builder.RegisterParams();
                builder.RegisterCommand();
                builder.RegisterCommandIntEx();
                builder.RegisterCommandLongEx();
                builder.RegisterMission();
                builder.RegisterMissionEx();
                builder.RegisterStatus();

                builder.RegisterMode(
                    ArduCopterMode.PosHold,
                    SupportedModes,
                    mode => new RoverModeHandler(Log, mode)
                );                
                             
                //builder.RegisterFtpEx();      // File transfer
                //builder.RegisterGbsEx();        //GBS (Ground Control Station)   
                builder.RegisterDiagnostic();

                //builder.RegisterRsgaEx();     //RSGA (Radiosonde Ground Apparatus) --  radio track points?
                //builder.RegisterV2Ext();
                //builder.Register<DgpsServer>();                
                //builder.Register<ControlServer>();    // Currently unavailable in asv-mavlink
                //builder.Register<TelemetryServer>();  // Currently unavailable in asv-mavlink
            });

        await SetupHeartbeatService();
        await SetupParamsService();
        await SetupCommandService();
        //await SetupMission();
        await SetupMissionService();
        await SetupStatusService();
        await SetupModeService();

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
        // TODO: properly dispose all subscriptions and resources, including Setup*() method calls
    }

    private IProtocolRouter SetupRouter(IProtocolFactory protocol)
    {
        Log.Info($"[MAVLink] Listening on {LocalHost}:{LocalPort} (Rover)");
        Log.Info($"[MAVLink] Sending to {RemoteHost}:{RemotePort} (GCS)");

        var router = protocol.CreateRouter("ROUTER");
        router.AddUdpPort(p =>
        {
            p.Query.Set("rhost", RemoteHost);
            p.Query.Set("rport", RemotePort.ToString());
            p.Host = LocalHost;
            p.Port = LocalPort;
        });

        foreach (var port in router.Ports)
        {
            port.EndpointAdded.Subscribe(ep =>
            {
                Log.Info($"[MAVLink] Endpoint added: {ep.DisplayName}");
            });
            port.EndpointRemoved.Subscribe(ep =>
            {
                Log.Info($"[MAVLink] Endpoint removed: {ep.DisplayName}");
            });
        }

        router.OnRxMessage.Subscribe(msg =>
        {
            if (MUTED_MESSAGES.Contains(msg.Name))
            {
                return;
            }
            if (msg is MavlinkV2Message mavlinkMsg && mavlinkMsg.FullId != _mavlinkId)
            {
                Log.Info($"[MAVLink] Received {mavlinkMsg.Name} from ({mavlinkMsg.SystemId}, {mavlinkMsg.ComponentId})");
            }
        });
        router.OnTxMessage.Subscribe(msg =>
        {
            if (msg is MavlinkV2Message mavlinkMsg && mavlinkMsg.FullId == _mavlinkId)
            {
                if (MUTED_MESSAGES.Contains(msg.Name))
                {
                    return;
                }
                else if (msg.Name == "STATUSTEXT" && msg is StatustextPacket statusMsg)
                {
                    Log.Info($"Sent status msg [{statusMsg.Payload.Severity}]: {string.Concat(statusMsg.Payload.Text)}");
                }
                Log.Info($"[MAVLink] Transmitted {msg.Name}");
            }
        });
        return router;
    }

    private async Task SetupParamsService()
    {
        // TODO: Use IParamsServerEx
        var paramsService = _serverDevice.GetParams();
        paramsService.OnParamRequestRead.Subscribe(request =>
        {
            var name = MavlinkTypesHelper.GetString(request.Payload.ParamId);
            Log.Info($"Parameter read requested: {name}");
        });
        paramsService.OnParamRequestList.Subscribe(async req =>
        {
            Log.Info("Parameter list requested");
            for (ushort i = 0; i < MAVLINK_PARAMETERS.Length; i++)
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
        paramsService.OnParamSet.Subscribe(packet =>
        {
            var name = MavlinkTypesHelper.GetString(packet.Payload.ParamId);
            Log.Info($"Parameter set: {name} = {packet.Payload.ParamValue}");
        });
        await paramsService.Init(_cts.Token);
        paramsService.Start();
    }

    private async Task SetupStatusService()
    {
        var statusService = _serverDevice.GetStatus();
        if (!statusService.IsInit) await statusService.Init(_cts.Token);
        //statusService.
        //statusService.Info("Rover is OK");
        statusService.Start();
    }

    private async Task SetupHeartbeatService()
    {
        var heartbeatService = _serverDevice.GetHeartbeat();
        heartbeatService.Set(hb =>
        {
            hb.Type = MavType.MavTypeGroundRover;
            // QGC has hardcoded mode flags from Ardupilot
            hb.Autopilot = MavAutopilot.MavAutopilotArdupilotmega;
            hb.BaseMode = MavModeFlag.MavModeFlagCustomModeEnabled;
            // Bit flags: can add e.g. | MavModeFlag.MavModeFlagSafetyArmed;
            hb.CustomMode = 4;
            hb.SystemStatus = MavState.MavStateActive;
            hb.MavlinkVersion = 3;
        });
        await heartbeatService.Init(_cts.Token);
        heartbeatService.Start();
    }

    private TPacket createPacket<TPacket, TPayload>(
        Action<TPayload, TransformComponent, RigidbodyComponent> packer)
        where TPacket : MavlinkV2Message<TPayload>, new()
        where TPayload : IPayload
    {
        // TODO: This should set seq, etc.? But not doing it. What's the point then?
        //var packet = _serverDevice.Context.MessageFactory.Create(GlobalPositionIntPacket.MessageId);
        //var payload = (GlobalPositionIntPayload)packet.GetPayload();

        var packet = new TPacket
        {
            SystemId = _mavlinkId.SystemId,
            ComponentId = _mavlinkId.ComponentId,
            Sequence = _serverDevice.Context.Sequence.GetNextSequenceNumber()
        };

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

    private async Task SetupCommandService()
    {
        var commandIntService = _serverDevice.GetCommandIntEx();
        commandIntService[MavCmd.MavCmdGetHomePosition] = async (id, packet, ct) =>
        {
            Log.Info("[MAVLink] Get home called");
            return CommandResult.Accepted;
        };
        commandIntService[MavCmd.MavCmdDoSetHome] = async (id, packet, ct) =>
        {
            Log.Info("[MAVLink] Set home called");
            return CommandResult.Accepted;
        };
        await commandIntService.Init(_cts.Token);
        commandIntService.Start();

        //var commandLongService = _serverDevice.GetCommandLongEx();
    }

    private async Task SetupMissionService()
    {
        var missionServiceEx = _serverDevice.GetMissionEx();
        
        var missionService = missionServiceEx.Base;
        //var missionService = _serverDevice.GetMission();

        // --- CHANGE ACTIVE ITEM ---
        // COMMAND_LONG (MAV_CMD_DO_SET_MISSION_CURRENT) -> COMMAND_ACK + MISSION_CURRENT (Broadcast)

        missionServiceEx[MavCmd.MavCmdDoSetMissionCurrent] = async (item, ct) =>
        {
            Log.Info($"Mission SET_CURRENT {item.Seq}");
            await missionService.SendMissionAck(MavMissionResult.MavMissionAccepted);
            await missionService.SendMissionCurrent(item.Seq);
        };

        await missionServiceEx.Init(_cts.Token);
        missionServiceEx.Start();
    }

    // Basically this is handled by the MissionServerEx
    // Let's save this code unused by now, just in case we'll need something more customizable later
    private async Task SetupMission()
    {
        // --- LOAD (GCS -> Rover) ---
        // MISSION_COUNT -> MISSION_REQUEST_INT -> [MISSION_ITEM_INT x N] -> MISSION_ACK (Rover)
        var missionService = _serverDevice.GetMission();
        await missionService.Init(_cts.Token);
        var mission = new ConcurrentDictionary<ushort, ServerMissionItem>();

        using var onMissionCount =
            missionService.OnMissionCount.SubscribeAwait(async (req, ct) =>
            {
                ushort itemsCount = req.Payload.Count;
                Log.Info($"Mission COUNT {itemsCount}");
                var missionType = req.Payload.MissionType;
                for (ushort i = 0; i < itemsCount; i++)
                {
                    var item = await missionService.RequestMissionItem(i, missionType);
                    Log.Info($"Mission item {i + 1}/{itemsCount} received");
                }
                await missionService.SendMissionAck(MavMissionResult.MavMissionAccepted);
                // TODO: Check that GCS and Rover COUNT match
            });

        using var onMissionRequest =
            missionService.OnMissionRequestInt.SubscribeAwait(async (req, ct) =>
            {
                Log.Info($"Mission REQUEST_INT {req.Payload.Seq}");
                mission.TryGetValue(req.Payload.Seq, out var item);
                if (item is null)
                {
                    Log.Warning($"Requested mission item {req.Payload.Seq} not found.");
                    await missionService.SendMissionAck(MavMissionResult.MavMissionInvalidSequence);
                    return;
                }
                await missionService.SendMissionItemInt(item, cancel: ct);
            });


        using var onMissionAck =
            missionService.OnMissionAck.SubscribeAwait(async (req, ct) =>
            {
                Log.Info("Mission ACK");
            });

        // --- DOWNLOAD (Rover -> GCS) ---
        // MISSION_REQUEST_LIST -> MISSION_COUNT -> [MISSION_REQUEST_INT x N] -> [MISSION_ITEM_INT x N] -> MISSION_ACK (GCS)

        using var onMissionList =
            missionService.OnMissionRequestList.SubscribeAwait(async (req, ct) =>
            {
                Log.Info("Mission LIST");
                await missionService.SendMissionCount((ushort)mission.Count);
                foreach (var item in mission.Values)
                {
                    // It's disadvised to send these concurrently
                    // TODO: specify target system id and component id to GCS
                    await missionService.SendMissionItemInt(item, cancel: ct);
                }
                await missionService.SendMissionAck(MavMissionResult.MavMissionAccepted);
            });


        // --- CLEAR ---
        // MISSION_CLEAR_ALL -> MISSION_ACK (Rover)

        using var onMissionClear =
            missionService.OnMissionClearAll.SubscribeAwait(async (req, ct) =>
            {
                Log.Info("Mission COUNT");
                //items.Clear();
                await missionService.SendMissionAck(
                    MavMissionResult.MavMissionAccepted,
                    req.SystemId,
                    req.ComponentId
                );
            });

        missionService.Start();
    }

    private async Task SetupModeService()
    {
        var modeService = _serverDevice.GetMode();
        if (!modeService.IsInit) await modeService.Init(_cts.Token);

        //await modeService.SetMode(ArduCopterMode.Loiter, handler =>
        //{
        //    Console.WriteLine("Mode change requested: Loiter");
        //    // optional: configure handler / initial state
        //});
        modeService.CurrentMode.Subscribe(modeHandler => { 
            Log.Info($"Current mode changed: {modeHandler.Mode.Name}"); 
        });

        modeService.Start();
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
        await _serverDevice.Context.Connection.Send(packet);
    }

    private async Task SendGlobalPosition()
    {
        var packet = createPacket<GlobalPositionIntPacket, GlobalPositionIntPayload>((payload, t, rb) =>
        {
            var pos = t.Position;
            // Breslau, Polen
            //int lat_offset = 511079000;
            //int lon_offset = 170385000;
            //int alt_offset = 118000;
            int lat_offset = 0;
            int lon_offset = 0;
            int alt_offset = 0;
            payload.Lat = lat_offset + (int)(-pos.Z * 100);
            payload.Lon = lon_offset + (int)(pos.X * 100);
            payload.Alt = alt_offset + (int)(-pos.Y * 1000);
            payload.RelativeAlt = payload.Alt;

            var v = rb.LinearVelocity;
            payload.Vx = (short)(-v.Z * 100);
            payload.Vy = (short)(v.X * 100);
            payload.Vz = (short)(-v.Y * 100);

            var forward = t.WorldMatrix.Forward;
            float theta = (float)Math.Atan2(forward.X, -forward.Z);
            float hdg = MathUtil.RadiansToDegrees(theta);
            hdg = (hdg % 360f + 360f) % 360f;
            payload.Hdg = (ushort)Math.Min(hdg * 100f, 35999f);

            payload.TimeBootMs = (uint)Game.UpdateTime.Total.TotalMilliseconds;
        });
        await _serverDevice.Context.Connection.Send(packet);
    }

    private async Task SendAttitude()
    {
        var packet = createPacket<AttitudePacket, AttitudePayload>((payload, t, rb) =>
        {
            var wm = t.WorldMatrix;
            var f = Vector3.Normalize(wm.Forward);
            var r = Vector3.Normalize(wm.Right);
            var u = Vector3.Normalize(wm.Up);
            payload.Yaw = MathF.Atan2(f.X, -f.Z);
            payload.Pitch = MathF.Asin(Math.Clamp(f.Y, -1f, 1f));
            payload.Roll = MathF.Atan2(-r.Y, u.Y);

            var w = rb.AngularVelocity;
            payload.Yawspeed = -Vector3.Dot(w, u);
            payload.Pitchspeed = Vector3.Dot(w, r);
            payload.Rollspeed = Vector3.Dot(w, f);

            payload.TimeBootMs = (uint)Game.UpdateTime.Total.TotalMilliseconds;
        });
        await _serverDevice.Context.Connection.Send(packet);
    }

}