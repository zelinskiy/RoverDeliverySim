using Asv.Common;
using Asv.IO;
using Asv.Mavlink;
using Asv.Mavlink.Ardupilotmega;
using Asv.Mavlink.Common;
using Asv.Mavlink.Diagnostic;
using Asv.Mavlink.Minimal;
using DotNext;
using Geodesy;
using R3;
using RoverDeliverySim.MavlinkExtensions;
using Stride.Core;
using Stride.Core.Diagnostics;
using Stride.Core.Mathematics;
using Stride.Engine;
using Stride.Physics;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using System.Timers;
using static System.Net.Mime.MediaTypeNames;
using MavCmd = Asv.Mavlink.Common.MavCmd;

namespace RoverDeliverySim;



public class RoverMavLinkConnection : AsyncScript
{
    // TODO: 
    // - Handle SYSTEM_TIME
    // - Handle MSG_SET_INTERVAL: Param1 = 242, 245
    // mavproxy --master=udpin:127.0.0.1:14550 --out=udpout:192.168.1.50:14560 [--no-console]
    public int RemotePort = 14550;
    public string RemoteHost = "127.0.0.1";
    public int LocalPort = 14560;
    public string LocalHost = "127.0.0.1";
    private IServerDevice _serverDevice;
    // Should be available like shown below, but unfortunately it doesn't :(
    //var hbConfig = _serverDevice.Configuration.Get<MavlinkHeartbeatServerConfig>(
    //    nameof(MavlinkHeartbeatServerConfig), 
    //    new (() => throw new InvalidOperationException("Missing heartbeat config"))
    //);
    private MavlinkHeartbeatServerConfig _heartbeatConfig;
    private CancellationTokenSource _cts;
    private static readonly MavlinkIdentity MavlinkId = new(1, 1);
    // To disable single sensor:
    // var sensors = Sensors & ~MavSysStatusSensor.MavSysStatusSensorGps;
    private static readonly MavSysStatusSensor Sensors =
        (MavSysStatusSensor.MavSysStatusPrearmCheck    // Skip arming checks in QGC
        | MavSysStatusSensor.MavSysStatusSensor3dGyro
        | MavSysStatusSensor.MavSysStatusSensor3dAccel
        | MavSysStatusSensor.MavSysStatusSensor3dMag
        //| MavSysStatusSensor.MavSysStatusSensorGps
        | MavSysStatusSensor.MavSysStatusSensorMotorOutputs
        | MavSysStatusSensor.MavSysStatusSensorRcReceiver
        );
    private static readonly MavProtocolCapability Capabilities =
        (MavProtocolCapability.MavProtocolCapabilityMissionFloat
        | MavProtocolCapability.MavProtocolCapabilityParamFloat
        | MavProtocolCapability.MavProtocolCapabilityMissionInt
        | MavProtocolCapability.MavProtocolCapabilityCommandInt
        | MavProtocolCapability.MavProtocolCapabilityMavlink2
        //| MavProtocolCapability.MavProtocolCapabilityFtp
        );
    private readonly string[] MUTED_MESSAGES =
    {
        "ATTITUDE",
        "HEARTBEAT",
        "GLOBAL_POSITION_INT",
        "LOCAL_POSITION_NED",
        "SYS_STATUS",
        "REQUEST_DATA_STREAM",
        "FILE_TRANSFER_PROTOCOL",
    };

    private static readonly (string, float, MavParamType)[] MAVLINK_PARAMETERS =
    [
        ("SYSID_THISMAV", MavlinkId.SystemId, MavParamType.MavParamTypeUint8),
        ("FRAME_CLASS", 1f, MavParamType.MavParamTypeUint8), // Rover
        ("ARMING_CHECK", 0f, MavParamType.MavParamTypeInt32), // Only for testing!
        ("CRUISE_SPEED", 2f, MavParamType.MavParamTypeReal32), //TBD
        ("WP_RADIUS", 2f, MavParamType.MavParamTypeReal32), //TBD

        // Hardcoded from Ardupilot
        ("FLTMODE1", (float) RoverMode.RoverModeManual, MavParamType.MavParamTypeUint8),
        ("FLTMODE2", (float) RoverMode.RoverModeHold, MavParamType.MavParamTypeUint8),
        ("FLTMODE3", (float) RoverMode.RoverModeGuided, MavParamType.MavParamTypeUint8),
        ("FLTMODE4", (float) RoverMode.RoverModeAuto, MavParamType.MavParamTypeUint8),
        ("FLTMODE5", (float) RoverMode.RoverModeSmartRtl, MavParamType.MavParamTypeUint8),
        ("FLTMODE6", (float) RoverMode.RoverModeFollow, MavParamType.MavParamTypeUint8),
        ("GUID_SPEED", 2f, MavParamType.MavParamTypeReal32),
        ("GUID_TIMEOUT", 3f, MavParamType.MavParamTypeReal32),

        // Manual RC mapping
        ("RCMAP_ROLL", 1f, MavParamType.MavParamTypeUint8),
        ("RCMAP_PITCH", 2f, MavParamType.MavParamTypeUint8),
        ("RCMAP_THROTTLE", 3f, MavParamType.MavParamTypeUint8),
        ("RCMAP_YAW", 4f, MavParamType.MavParamTypeUint8),

        // Channel PWM Calibration
        // QGC will consider RC as uncalibrated if set these to 0 or defaults
        ("RC1_MIN", 1000f, MavParamType.MavParamTypeInt32),
        ("RC1_MAX", 1950f, MavParamType.MavParamTypeInt32),
        ("RC1_TRIM", 1500f, MavParamType.MavParamTypeInt32),
        ("RC1_REV", 1f, MavParamType.MavParamTypeInt32),

        ("RC2_MIN", 1000f, MavParamType.MavParamTypeInt32),
        ("RC2_MAX", 1950f, MavParamType.MavParamTypeInt32),
        ("RC2_TRIM", 1500f, MavParamType.MavParamTypeInt32),
        ("RC2_REV", 1f, MavParamType.MavParamTypeInt32),

        ("RC3_MIN", 1000f, MavParamType.MavParamTypeInt32),
        ("RC3_MAX", 1950f, MavParamType.MavParamTypeInt32),
        ("RC3_TRIM", 1500f, MavParamType.MavParamTypeInt32),
        ("RC3_REV", 1f, MavParamType.MavParamTypeInt32),

        ("RC4_MIN", 1000f, MavParamType.MavParamTypeInt32),
        ("RC4_MAX", 1950f, MavParamType.MavParamTypeInt32),
        ("RC4_TRIM", 1500f, MavParamType.MavParamTypeInt32),
        ("RC4_REV", 1f, MavParamType.MavParamTypeInt32),

        // Failsafe Configuration
        ("FS_OPTIONS", 0f, MavParamType.MavParamTypeInt32),   // Bitmask, each bit = mode
        ("FS_GCS_ENABLE", 1f, MavParamType.MavParamTypeInt32),
        ("FS_GCS_TIMEOUT", 5f, MavParamType.MavParamTypeReal32),
        
        // Accelerometer stuff
        ("INS_ACCL_ID", 67f, MavParamType.MavParamTypeInt32),  // whatever
        ("INS_ACCL_0_ID", 68f, MavParamType.MavParamTypeInt32),  // whatever
        ("INS_ACCOFFS_X", 0.01f, MavParamType.MavParamTypeReal32), // non-zero to pass calibration
        ("INS_ACCOFFS_Y", 0.01f, MavParamType.MavParamTypeReal32), // non-zero to pass calibration
        ("INS_ACCOFFS_Z", 0.01f, MavParamType.MavParamTypeReal32), // non-zero to pass calibration
        ("INS_POS1_X", 0f, MavParamType.MavParamTypeReal32),
        ("INS_POS1_Y", 0f, MavParamType.MavParamTypeReal32),
        ("INS_POS1_Z", 0f, MavParamType.MavParamTypeReal32),

        // Compass stuff 
        ("COMPASS_DEV_ID", 1024f, MavParamType.MavParamTypeInt32), // whatever
        ("COMPASS_DEV_ID2", 0f, MavParamType.MavParamTypeInt32),
        ("COMPASS_DEV_ID3", 0f, MavParamType.MavParamTypeInt32),

        ("COMPASS_USE", 1f, MavParamType.MavParamTypeInt32),

        ("COMPASS_OFS_X", 5.0f, MavParamType.MavParamTypeReal32), // non-zero to pass calibration
        ("COMPASS_OFS_Y", 10.0f, MavParamType.MavParamTypeReal32), // non-zero to pass calibration
        ("COMPASS_OFS_Z", -15.0f, MavParamType.MavParamTypeReal32), // non-zero to pass calibration

        ("COMPASS_OFS2_X", 1f, MavParamType.MavParamTypeReal32),
        ("COMPASS_OFS2_Y", 1f, MavParamType.MavParamTypeReal32),
        ("COMPASS_OFS2_Z", 1f, MavParamType.MavParamTypeReal32),

        ("COMPASS_OFS3_X", 0f, MavParamType.MavParamTypeReal32),
        ("COMPASS_OFS3_Y", 0f, MavParamType.MavParamTypeReal32),
        ("COMPASS_OFS3_Z", 0f, MavParamType.MavParamTypeReal32),

        ("COMPASS_DIA_X", 0f, MavParamType.MavParamTypeReal32),
        ("COMPASS_DIA_Y", 0f, MavParamType.MavParamTypeReal32),
        ("COMPASS_DIA_Z", 0f, MavParamType.MavParamTypeReal32),

        ("COMPASS_DEC", 0f, MavParamType.MavParamTypeReal32),  // Magnetic declination (radians)
        ("COMPASS_AUTODEC", 1f, MavParamType.MavParamTypeInt32),

        // Battery Stuff
        ("BATT_MONITOR", 0f, MavParamType.MavParamTypeUint8),   // Disabled to prevent low voltage pre-arm block
        ("BATT_VOLT_MSRC", 12600f, MavParamType.MavParamTypeReal32),

        // Gyroscope Stuff
        ("INS_GYRO_ID", 42f, MavParamType.MavParamTypeReal32),
        ("INS_GYRO_0_ID", 43f, MavParamType.MavParamTypeReal32),

        ("AHRS_ORIENTATION", 0f,   MavParamType.MavParamTypeInt32),
    ];

    private new readonly Logger Log = GlobalLogger.GetLogger("MAVLink");
    private static readonly List<ArduRoverMode> SupportedModes =
    [ ArduRoverMode.Manual
    //, ArduRoverMode.Steering, do we need it?
    , ArduRoverMode.Hold
    , ArduRoverMode.Guided
    , ArduRoverMode.Auto
    , ArduRoverMode.Follow
    , ArduRoverMode.Rtl
    , ArduRoverMode.SmartRtl
    //, ArduRoverMode.Initializing, do we need it??
    ];

    // TODO: Encapsulate these two collections into one entity somehow
    // These are handled inside Stride Execute() loop
    private readonly TimerCollection<int, RoverMavLinkConnection> messageTimers =
        new(TimeSpan.FromSeconds(1))
    {
        { ExtendedSysStatePacket.MessageId },
        { LocalPositionNedPacket.MessageId, c => c.SendLocalPosition() },
        { GlobalPositionIntPacket.MessageId, c => c.SendGlobalPosition() },
        { AttitudePacket.MessageId, c => c.SendAttitude() },
        { SysStatusPacket.MessageId, c => c.SendSysStatus() },
    };
    // Update called once and basically handled by services
    private readonly Dictionary<int, Func<RoverMavLinkConnection,TimeSpan, Task>> setMsgIntervalHandlers = new()
    {
        [HomePositionPacket.MessageId] = (c, i) => Task.CompletedTask,
        [HeartbeatPacket.MessageId] = (c, i) => c.UpdateHeartbeatInterval(i),
    };

    private class RoverModeHandler : IWorkModeHandler
    {
        private readonly ArduRoverMode _mode;

        public RoverModeHandler(Logger log, ICustomMode mode)
        {
            Log = log;
            if (mode is ArduRoverMode roverMode)
            {
                _mode = roverMode;
            }
            else
            {
                throw new ArgumentException($"Expected ArduRoverMode, got {mode?.GetType().Name}");
            }
        }
        public ArduRoverMode Mode => _mode;
        ICustomMode IWorkModeHandler.Mode => _mode;
        public Logger Log { get; }
        public Task Init(CancellationToken cancel)
        {
            Log.Info($"Handler for {_mode.Name} mode created");
            //if (mode == ArduCopterMode.Guided)
            //{
            //    Log.Info($"Handler for Guided mode created");
            //}
            return Task.CompletedTask;
        }

        public Task Destroy(CancellationToken cancel)
        {
            Log.Info($"Handler for {_mode.Name} mode destroyed");
            //if (mode == ArduCopterMode.Guided)
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
            MavlinkId,
            core,
            builder =>
            {
                _heartbeatConfig = new MavlinkHeartbeatServerConfig();
                UpdateHeartbeatInterval(TimeSpan.FromSeconds(1));
                builder.RegisterHeartbeat(_heartbeatConfig);
                builder.RegisterParams();
                builder.RegisterCommand();
                builder.RegisterCommandIntEx();
                builder.RegisterCommandLongEx();
                builder.RegisterMission();
                builder.RegisterMissionEx();
                builder.RegisterStatus();

                // Can't use the standard mode microservice since it got mode mapping implemented
                // only for Px4 and ArduCopter, and we need ArduPilotMega Drones.
                // TODO: Perhaps make our own microservice OR implement RoverMode : ICustomMode?

                builder.RegisterMode(
                    ArduRoverMode.Hold,
                    SupportedModes,
                    mode => new RoverModeHandler(Log, mode)
                );

                //builder.RegisterFtpEx(); // File transfer
                //builder.RegisterGbsEx(); //GBS (Ground Control Station)   
                builder.RegisterDiagnostic();
                //builder.Register((id, _, _) => new DiagnosticServer(id, new DiagnosticServerConfig(), core));

                //RSGA (Radiosonde Ground Apparatus) -- e.g. radio track points?
                //builder.RegisterRsgaEx();     
                //builder.RegisterV2Ext();
                //builder.Register<DgpsServer>();
                // Currently unavailable in asv-mavlink:
                //builder.Register<ControlServer>();
                // Currently unavailable in asv-mavlink:
                //builder.Register<TelemetryServer>();
            });

        await SetupHeartbeatService();
        await SetupParamsService();
        await SetupCommandService();
        await SetupMissionService();
        await SetupStatusService();
        await SetupModeService();
        await SetupDiagnostic();
        //await SetupMission(); -- already handled by MissionEx
        //await SetupFileTransfer()

        // TODO: Match frequencies:
        // HEARTBEAT, SYS_STATUS: 1 Hz
        // GLOBAL_POSITION_INT, LOCAL_POSITION_NED: 2-5 Hz
        // ATTITUDE: 10-20 Hz

        while (Game.IsRunning && !_cts.IsCancellationRequested)
        {
            await messageTimers.TickAsync(Game.UpdateTime.Total, this);
            await Script.NextFrame();
        }

        await _cts.CancelAsync();
        //_router.Dispose(); TBD ???
        // TODO: properly dispose all subscriptions and resources, including Setup*() method calls
        // TODO: Check _cts is propagated everywhere
    }

    private Task UpdateHeartbeatInterval(TimeSpan interval)
    {
        _heartbeatConfig.HeartbeatRateMs = (int)interval.TotalMilliseconds;
        return Task.CompletedTask;
    }

    private async Task SetupDiagnostic()
    {
        //DiagnosticServer diagnosticService = _serverDevice.Get<DiagnosticServer>();
        var diagnosticService = _serverDevice.GetDiagnostic();
        if (!diagnosticService.IsInit) await diagnosticService.Init(_cts.Token);

        //diagnosticService.Send("")

        diagnosticService.Start();
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
                if (ep.GetPortId() == LocalPort.ToString()) return;
                Log.Info($"[MAVLink] Endpoint added: {ep.DisplayName}");
            });
            port.EndpointRemoved.Subscribe(ep =>
            {
                if (ep.GetPortId() == LocalPort.ToString()) return;
                Log.Info($"[MAVLink] Endpoint removed: {ep.DisplayName}");
            });
        }

        router.OnRxMessage.Subscribe(async msg =>
        {
            // TODO: Pay attention to SystemId/ComponentId
            if (MUTED_MESSAGES.Contains(msg.Name))
            {
                return;
            }
            if (msg is MavlinkV2Message mavlinkMsg && mavlinkMsg.FullId != MavlinkId)
            {
                Log.Info($"[MAVLink] Received {mavlinkMsg.Name} from ({mavlinkMsg.SystemId}, {mavlinkMsg.ComponentId})");
                if (msg is FileTransferProtocolPacket ftpPacket)
                {
                    // Unfortunately suppressing
                    // Construct an explicit NAK response indicating failure/unsupported session
                    var responsePacket = new FileTransferProtocolPacket();
                    responsePacket.Payload.TargetSystem = ftpPacket.SystemId;
                    responsePacket.Payload.TargetComponent = ftpPacket.ComponentId;
                    responsePacket.Payload.TargetNetwork = ftpPacket.Payload.TargetNetwork;

                    // Byte 0: 0x84 (NAK response), Byte 2: Error code (e.g., Fail = 1 or NoSessionsAvailable = 3)
                    responsePacket.Payload.Payload[0] = 0x84; // NAK
                    responsePacket.Payload.Payload[2] = 3;   // Fail / Unsupported error code

                    await _serverDevice.Context.Connection.Send(responsePacket);
                }
                else if (msg is CommandLongPacket cmdLong)
                {
                    Log.Info($"[MAVLink] Mavlink CMD_LONG: {cmdLong.Payload.Command}, " +
                        $"Params: {cmdLong.Payload.Param1}:" +
                        $"{cmdLong.Payload.Param2}:" +
                        $"{cmdLong.Payload.Param3}:" +
                        $"{cmdLong.Payload.Param4}:" +
                        $"{cmdLong.Payload.Param5}:" +
                        $"{cmdLong.Payload.Param6}:" +
                        $"{cmdLong.Payload.Param7}:");
                }
                else if (msg is CommandIntPacket cmdInt)
                {
                    Log.Info($"[MAVLink] Mavlink CMD_INT: {cmdInt.Payload.Command}, " +
                        $"Params: {cmdInt.Payload.Param1}:" +
                        $"{cmdInt.Payload.Param2}:" +
                        $"{cmdInt.Payload.Param3}:" +
                        $"{cmdInt.Payload.Param4}");
                }
            }
        });
        router.OnTxMessage.Subscribe(msg =>
        {
            if (msg is MavlinkV2Message mavlinkMsg && mavlinkMsg.FullId == MavlinkId)
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

    private async Task SetupFileTransfer()
    {
        var ftpService = _serverDevice.GetFtpEx();
        if (!ftpService.IsInit) await ftpService.Init(_cts.Token);
        ftpService.Start();
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
        if (!heartbeatService.IsInit) await heartbeatService.Init(_cts.Token);
        heartbeatService.Set(hb =>
        {
            var modeService = _serverDevice.GetMode();
            modeService.CurrentMode.CurrentValue.Mode.GetCommandLongArgs(
                out var baseMode,
                out var customMode,
                out var customSubMode
            );
            modeService.CurrentMode.CurrentValue.Mode.Fill(hb);
            //Log.Info($"Current mode: {modeService.CurrentMode.CurrentValue.Mode.Name}; {baseMode}:{customMode}");

            //MavMode.
            hb.Type = MavType.MavTypeGroundRover;
            // QGC has hardcoded mode flags from Ardupilot
            //hb.Autopilot = MavAutopilot.MavAutopilotArdupilotmega;
            hb.Autopilot = MavAutopilot.MavAutopilotArdupilotmega;
            //hb.BaseMode = (MavModeFlag)baseMode;
            //hb.BaseMode = MavModeFlag.MavModeFlagCustomModeEnabled;
            // Bit flags: can add e.g. | MavModeFlag.MavModeFlagSafetyArmed;
            //hb.CustomMode = 4;
            hb.SystemStatus = MavState.MavStateActive;
            hb.MavlinkVersion = 3;
        });
        _serverDevice.GetCommand();       
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
            SystemId = MavlinkId.SystemId,
            ComponentId = MavlinkId.ComponentId,
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
        if (!commandIntService.IsInit) await commandIntService.Init(_cts.Token);
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
        commandIntService.Start();

        var commandLongService = _serverDevice.GetCommandLongEx();
        if (!commandLongService.IsInit) await commandLongService.Init(_cts.Token);
        commandLongService[MavCmd.MavCmdRequestMessage] = async (id, packet, ct) =>
        {
            var messageId = packet.Payload.Param1;
            switch (messageId)
            {
                case AutopilotVersionPacket.MessageId:
                    Log.Info("[MAVLink] Requested Autopilot Version");
                    await SendAutopilotVersion();
                    return CommandResult.Accepted;
                case 395: // URI pointing to a JSON metadata (old)
                    return CommandResult.Unsupported;
                case 397: // URI pointing to a JSON metadata (recent)
                    return CommandResult.Unsupported;
                case 280: // GIMBAL_MANAGER_INFORMATION
                    return CommandResult.Unsupported;
                case 435: // AVAILABLE_MODES
                    await SendAvailableModes();
                    return CommandResult.Accepted;
                default:
                    Log.Info($"Requested unhandled message {messageId}");
                    return CommandResult.Unsupported;
            }

        };
        commandLongService[MavCmd.MavCmdRequestAutopilotCapabilities] = async (id, packet, ct) =>
        {
            await SendAutopilotVersion();
            return CommandResult.Accepted;
        };
        commandLongService[MavCmd.MavCmdDoSetMode] = async (id, packet, ct) =>
        {
            Log.Info("[MAVLink] Set mode called (LONG)");
            var modeService = _serverDevice.GetMode();
            var mode_code = (int)packet.Payload.Param2;
            var mode = ArduRoverMode.FromInt(mode_code);
            if (mode is null || !modeService.AvailableModes.Contains(mode))
            {
                Log.Info($"[MAVLink] Bad mode: {mode_code}");
                return CommandResult.Denied;
            }
            // From the docs (ModeServer.md):
            // > The current `ModeServer.SetMode` implementation has two known limitations:
            // after a different mode handler is initialized successfully, it is not assigned to `CurrentMode`, 
            // so the previous mode remains in outgoing heartbeats;
            // Which is quite bizarre TBH
            await modeService.SetMode(mode, null, _cts.Token);

            // NB: hbService.SetCustomMode() is a utility function used
            // with firmwares that pack multiple bits inside CustomMode msg, e.g. PX4.
            var hbService = _serverDevice.GetHeartbeat();
            hbService.Set(mode.Fill);
            Log.Info($"[MAVLink] Changed mode to {mode.Name}");
            return CommandResult.Accepted;
        };
        commandLongService[MavCmd.MavCmdSetMessageInterval] = async (id, packet, ct) =>
        {
            var messageId = (int)packet.Payload.Param1;
            var intervalMs = (long)packet.Payload.Param2;
            if (messageTimers.Contains(messageId))
            {
                messageTimers.PostIntervalMicroseconds(messageId, intervalMs);
                return CommandResult.Accepted;
            } 
            else if (setMsgIntervalHandlers.TryGetValue(messageId, out var handler))
            {
                await handler(this, TimeSpan.FromMilliseconds(intervalMs));
                return CommandResult.Accepted;
            }
            else
            {
                Log.Warning($"Requested to set interval for unsupported message {messageId}");
                return CommandResult.Unsupported;
            }
            
        };
        commandLongService.Start();
    }

    private async Task SendAvailableModes()
    {
        var modeService = _serverDevice.GetMode();
        List<ArduRoverMode> modes = SupportedModes;

        for (int i = 0; i < SupportedModes.Count; i++)
        {
            var mode = SupportedModes[i];
            var packet = createPacket<AvailableModesPacket, AvailableModesPayload>((payload, t, rb) =>
            {
                payload.StandardMode = mode.ToStandardMode();
                payload.CustomMode = (uint)mode.CustomMode;
                payload.ModeIndex = (byte)(i + 1);
                payload.NumberModes = (byte)modes.Count;
                payload.Properties = 0;

                Array.Clear(payload.ModeName, 0, payload.ModeName.Length);
                System.Text.Encoding.ASCII.GetBytes(mode.Name).AsSpan().CopyTo(payload.ModeName);
            });

            await _serverDevice.Context.Connection.Send(packet);
        }
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
        modeService.CurrentMode.Subscribe(modeHandler =>
        {
            Log.Info($"Current mode changed: {modeHandler.Mode.Name}");
        });

        modeService.Start();
        var modesList = string.Join(", ", modeService.AvailableModes.Select(m => m.Name));
        Log.Info($"Started modes microservice. Supported modes: {modesList}");
    }

    protected async Task SendLocalPosition()
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

    protected async Task SendGlobalPosition()
    {
        var packet = createPacket<GlobalPositionIntPacket, GlobalPositionIntPayload>((payload, t, rb) =>
        {
            var pos = t.Position;
            payload.Lat = (int)(SaveMapImageScript.SceneOriginLat * Math.Pow(10, 7) - pos.Z * 100);
            payload.Lon = (int)(SaveMapImageScript.SceneOriginLon * Math.Pow(10, 7) + pos.X * 100);
            payload.Alt = (int)(SaveMapImageScript.SceneOriginAlt * 1000 - pos.Y * 1000);
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

    protected async Task SendAttitude()
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

    protected async Task SendSysStatus()
    {
        var packet = createPacket<SysStatusPacket, SysStatusPayload>((payload, t, rb) =>
        {
            payload.OnboardControlSensorsPresent = Sensors;
            payload.OnboardControlSensorsEnabled = Sensors;
            payload.OnboardControlSensorsHealth = Sensors;
            payload.VoltageBattery = 12600; // V * 10^(-3)
            payload.CurrentBattery = 150;   // A * 10^(-2)
            payload.BatteryRemaining = 100; // 100%
            payload.Load = 50;              // 5% CPU load
            payload.DropRateComm = 0;       // 0% packet drop
            payload.ErrorsComm = 0;         // 0 link errors
            payload.ErrorsCount1 = 0;
            payload.ErrorsCount2 = 0;
            payload.ErrorsCount3 = 0;
            payload.ErrorsCount4 = 0;
        });
        await _serverDevice.Context.Connection.Send(packet);
    }

    protected async Task SendAutopilotVersion()
    {
        var packet = createPacket<AutopilotVersionPacket, AutopilotVersionPayload>((payload, t, rb) =>
        {
            payload.Capabilities = Capabilities;
            // Bypassing qgc irrelevant firmware warning:
            payload.FlightSwVersion = 0x04070200;
            payload.Uid = 123456789L;
            payload.BoardVersion = 0;
            payload.VendorId = 0;
            payload.ProductId = 0;

        });
        await _serverDevice.Context.Connection.Send(packet);
    }
}