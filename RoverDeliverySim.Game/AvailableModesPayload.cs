using System;
using System.Runtime.CompilerServices;
using Asv.Mavlink;
using Asv.IO;

namespace RoverDeliverySim.MavlinkExtensions;

public class AvailableModesPayload : IPayload, ISizedSpanSerializable, ISpanSerializable, IVisitable
{
    public static readonly Field CustomModeField = new Field.Builder().Name("CustomMode").Title("custom_mode").Description("A bitfield for use for autopilot-specific flags")
        .DataType(UInt32Type.Default)
        .Build();

    private uint _customMode;

    public static readonly Field PropertiesField = new Field.Builder().Name("Properties").Title("properties").Description("Mode properties")
        .DataType(UInt32Type.Default)
        .Build();

    private uint _properties;

    public static readonly Field NumberModesField = new Field.Builder().Name("NumberModes").Title("number_modes").Description("The total number of available modes for the current vehicle type")
        .DataType(UInt8Type.Default)
        .Build();

    private byte _numberModes;

    public static readonly Field ModeIndexField = new Field.Builder().Name("ModeIndex").Title("mode_index").Description("The current mode index within number_modes, indexed from 1")
        .DataType(UInt8Type.Default)
        .Build();

    private byte _modeIndex;

    public static readonly Field StandardModeField = new Field.Builder().Name("StandardMode").Title("standard_mode").Description("Standard mode")
        .DataType(UInt8Type.Default)
        .Build();

    private byte _standardMode;

    public static readonly Field ModeNameField = new Field.Builder().Name("ModeName").Title("mode_name").Description("Mode metadata key or name")
        .DataType(new ArrayType(UInt8Type.Default, 35))
        .Build();

    public const int ModeNameMaxItemsCount = 35;

    public uint CustomMode
    {
        get => _customMode;
        set => _customMode = value;
    }

    public uint Properties
    {
        get => _properties;
        set => _properties = value;
    }

    public byte NumberModes
    {
        get => _numberModes;
        set => _numberModes = value;
    }

    public byte ModeIndex
    {
        get => _modeIndex;
        set => _modeIndex = value;
    }

    public byte StandardMode
    {
        get => _standardMode;
        set => _standardMode = value;
    }

    public byte[] ModeName { get; } = new byte[35];

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public byte GetMaxByteSize()
    {
        return 47;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public byte GetMinByteSize()
    {
        return 47;
    }

    [MethodImpl(MethodImplOptions.AggressiveOptimization)]
    public int GetByteSize()
    {
        return 47;
    }

    public void Deserialize(ref ReadOnlySpan<byte> buffer)
    {
        CustomMode = BinSerialize.ReadUInt(ref buffer);
        Properties = BinSerialize.ReadUInt(ref buffer);
        NumberModes = BinSerialize.ReadByte(ref buffer);
        ModeIndex = BinSerialize.ReadByte(ref buffer);
        StandardMode = BinSerialize.ReadByte(ref buffer);

        for (int i = 0; i < ModeName.Length; i++)
        {
            ModeName[i] = BinSerialize.ReadByte(ref buffer);
        }
    }

    public void Serialize(ref Span<byte> buffer)
    {
        BinSerialize.WriteUInt(ref buffer, CustomMode);
        BinSerialize.WriteUInt(ref buffer, Properties);
        BinSerialize.WriteByte(ref buffer, NumberModes);
        BinSerialize.WriteByte(ref buffer, ModeIndex);
        BinSerialize.WriteByte(ref buffer, StandardMode);

        for (int i = 0; i < ModeName.Length; i++)
        {
            BinSerialize.WriteByte(ref buffer, ModeName[i]);
        }
    }

    public void Accept(IVisitor visitor)
    {
        FieldType<UInt32Type, uint>.Accept(visitor, CustomModeField, ref _customMode);
        FieldType<UInt32Type, uint>.Accept(visitor, PropertiesField, ref _properties);
        FieldType<UInt8Type, byte>.Accept(visitor, NumberModesField, ref _numberModes);
        FieldType<UInt8Type, byte>.Accept(visitor, ModeIndexField, ref _modeIndex);
        FieldType<UInt8Type, byte>.Accept(visitor, StandardModeField, ref _standardMode);
        ArrayType.Accept(visitor, ModeNameField, delegate (int index, IVisitor v, Field f, IFieldType t)
        {
            FieldType<UInt8Type, byte>.Accept(v, f, t, ref ModeName[index]);
        });
    }

    [Obsolete("This method is deprecated. Use GetModeNameMaxItemsCount instead.")]
    public byte GetModeNameMaxItemsCount()
    {
        return 35;
    }
}