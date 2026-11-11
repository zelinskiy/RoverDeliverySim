using System;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.Linq;
using Asv.Mavlink.Ardupilotmega;
using Asv.Mavlink.Common;
using Asv.Mavlink.Minimal;
using Asv.Mavlink;
using DotNext.Collections.Generic;

#nullable enable

namespace RoverDeliverySim.MavlinkExtensions;

// TODO: Pull request to asv-mavlink/src/Asv.Mavlink/Devices/Client/Vehicles/Ardu/
public class ArduRoverMode : ICustomMode
{
    #region Static

    public static ImmutableArray<ArduRoverMode> AllModes { get; }

    static ArduRoverMode()
    {
        var wellKnownModes = new HashSet<RoverMode>(WellKnownRoverModes.Select(m => m.CustomMode));
        var allModes = new List<ArduRoverMode>();
        allModes.AddRange(WellKnownRoverModes);
        foreach (var mode in Enum.GetValues<RoverMode>())
        {
            if (wellKnownModes.Contains(mode)) continue;
            allModes.Add(new ArduRoverMode(mode.ToString("G"), string.Empty, true, MavModeFlag.MavModeFlagCustomModeEnabled, mode));
        }
        AllModes = [.. allModes];
    }

    private static IEnumerable<ArduRoverMode> WellKnownRoverModes
    {
        get
        {
            yield return Manual;
            yield return Acro;
            yield return Steering;
            yield return Hold;
            yield return Loiter;
            yield return Follow;
            yield return Simple;
            yield return Auto;
            yield return Rtl;
            yield return SmartRtl;
            yield return Guided;
            yield return Initializing;
        }
    }

    public static ArduRoverMode? FromInt(int n)
    {
        return AllModes.FirstOrDefault(m => (ulong)m.CustomMode == (ulong)n);
    }
    public byte ToStandardMode()
    {
        return CustomMode switch
        {
            RoverMode.RoverModeAuto     => 6, // Auto       -> MISSION
            RoverMode.RoverModeGuided   => 6, // Guided     -> MISSION
            RoverMode.RoverModeRtl      => 5, // RTL        -> SAFE_RECOVERY 
            RoverMode.RoverModeSmartRtl => 5, // Smart RTL  -> SAFE_RECOVERY
            RoverMode.RoverModeHold     => 1, // Hold       -> POSITION_HOLD
            RoverMode.RoverModeLoiter   => 1, // Loiter     -> POSITION_HOLD
            _                           => 0  // ...        -> NON_STANDARD
        };
    }


    public static readonly ArduRoverMode Unknown = new("Unknown", "Unknown rover mode", true, MavModeFlag.MavModeFlagCustomModeEnabled, (RoverMode)uint.MaxValue);
    /// <summary>
    /// Manual control passthrough
    /// </summary>
    public static readonly ArduRoverMode Manual = new("Manual", "Manual control surface movement, passthrough", false, MavModeFlag.MavModeFlagCustomModeEnabled, RoverMode.RoverModeManual);
    /// <summary>
    /// Acrobatic mode
    /// </summary>
    public static readonly ArduRoverMode Acro = new("Acro", "Rate controlled mode with no steering limits", false, MavModeFlag.MavModeFlagCustomModeEnabled, RoverMode.RoverModeAcro);
    /// <summary>
    /// Steering mode
    /// </summary>
    public static readonly ArduRoverMode Steering = new("Steering", "Direct steering angle control with speed scaling", false, MavModeFlag.MavModeFlagCustomModeEnabled, RoverMode.RoverModeSteering);
    /// <summary>
    /// Hold position / stop
    /// </summary>
    public static readonly ArduRoverMode Hold = new("Hold", "Stops the vehicle and holds position", false, MavModeFlag.MavModeFlagCustomModeEnabled, RoverMode.RoverModeHold);
    /// <summary>
    /// Loiter around a point
    /// </summary>
    public static readonly ArduRoverMode Loiter = new("Loiter", "Loiter around a point", false, MavModeFlag.MavModeFlagCustomModeEnabled, RoverMode.RoverModeLoiter);
    /// <summary>
    /// Follow target mode
    /// </summary>
    public static readonly ArduRoverMode Follow = new("Follow", "Follow a target leader vehicle or object", false, MavModeFlag.MavModeFlagCustomModeEnabled, RoverMode.RoverModeFollow);
    /// <summary>
    /// Simple mode relative to pilot heading
    /// </summary>
    public static readonly ArduRoverMode Simple = new("Simple", "Simple navigation relative to pilot heading", false, MavModeFlag.MavModeFlagCustomModeEnabled, RoverMode.RoverModeSimple);
    /// <summary>
    /// Autonomous mission following
    /// </summary>
    public static readonly ArduRoverMode Auto = new("Auto", "Follows Mission", false, MavModeFlag.MavModeFlagCustomModeEnabled, RoverMode.RoverModeAuto);
    /// <summary>
    /// Return to launch
    /// </summary>
    public static readonly ArduRoverMode Rtl = new("RTL", "Returns to and stops at home position", false, MavModeFlag.MavModeFlagCustomModeEnabled, RoverMode.RoverModeRtl);
    /// <summary>
    /// Smart return to launch
    /// </summary>
    public static readonly ArduRoverMode SmartRtl = new("Smart RTL", "Returns along the recorded path back home", false, MavModeFlag.MavModeFlagCustomModeEnabled, RoverMode.RoverModeSmartRtl);
    /// <summary>
    /// Guided mode from GCS
    /// </summary>
    public static readonly ArduRoverMode Guided = new("Guided", "Navigates to points defined from GCS", false, MavModeFlag.MavModeFlagCustomModeEnabled, RoverMode.RoverModeGuided);
    /// <summary>
    /// Initializing mode
    /// </summary>
    public static readonly ArduRoverMode Initializing = new("Initializing", "Vehicle is initializing", true, MavModeFlag.MavModeFlagCustomModeEnabled, RoverMode.RoverModeInitializing);

    #endregion

    public ArduRoverMode(string name, string description, bool internalMode, MavModeFlag mode, RoverMode customMode)
    {
        Mode = mode;
        CustomMode = customMode;
        Name = name;
        Description = description;
        InternalMode = internalMode;
    }

    public MavModeFlag Mode { get; }
    public RoverMode CustomMode { get; }
    public string Name { get; }
    public string Description { get; }
    public bool InternalMode { get; }

    public void GetCommandLongArgs(out uint baseMode, out uint customMode, out uint customSubMode)
    {
        baseMode = (uint)Mode;
        customMode = (uint)CustomMode;
        customSubMode = 0;
    }

    public bool IsCurrentMode(HeartbeatPayload? hb)
    {
        return hb?.CustomMode == (uint)CustomMode;
    }

    public bool IsCurrentMode(CommandLongPayload payload)
    {
        return (uint)payload.Param2 == (uint)CustomMode;
    }

    public void Fill(HeartbeatPayload hb)
    {
        hb.CustomMode = (uint)CustomMode;
        hb.BaseMode = Mode;
    }
}