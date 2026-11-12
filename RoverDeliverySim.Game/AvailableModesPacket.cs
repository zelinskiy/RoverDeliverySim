using Asv.Mavlink;
using Asv.Mavlink.Common;
using System;
using System.Runtime.CompilerServices;

namespace RoverDeliverySim.MavlinkExtensions;

public class AvailableModesPacket : MavlinkV2Message<AvailableModesPayload>
{
    public const int MessageId = 435;

    public const string MessageIdAsString = "435";

    public const byte CrcExtra = 216; // Standard CRC extra for AVAILABLE_MODES

    public override int Id => 435;

    public override bool WrapToV2Extension => false;

    public override AvailableModesPayload Payload { get; } = new AvailableModesPayload();

    public override string Name => "AVAILABLE_MODES";

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override string GetIdAsString()
    {
        return "435";
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override byte GetCrcExtra()
    {
        return 216;
    }
}
