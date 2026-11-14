using Stride.Core.Mathematics;
using System;
using System.Collections.Generic;
using System.Text;

namespace RoverDeliverySim;

internal static class PositionUtils
{
    public static readonly (double Lat, double Lon) SceneOriginGPS = (51.1079, 17.0385);
    public static readonly (double Lat, double Lon, double Alt) SceneOriginGPS3 = (SceneOriginGPS.Lat, SceneOriginGPS.Lon, 118.0);
    public static readonly (double Lat, double Lon, double Alt) SceneOriginMercator3;
    public static readonly (double Lat, double Lon) SceneOriginMercator;
    // Metres per degree at the origin latitude (WGS84 series)
    private static readonly (double Lat, double Lon) MetersPerDeg;

    static PositionUtils()
    {
        double phi = SceneOriginGPS.Lat * Math.PI / 180.0;
        MetersPerDeg = (
            111132.92 - 559.82 * Math.Cos(2 * phi) + 1.175 * Math.Cos(4 * phi),
            111412.84 * Math.Cos(phi) - 93.5 * Math.Cos(3 * phi)
        );
        SceneOriginMercator = LatLonToMercator(SceneOriginGPS.Lat, SceneOriginGPS.Lon);
        SceneOriginMercator3 = (SceneOriginMercator.Lat, SceneOriginMercator.Lon, SceneOriginGPS3.Alt);
    }

    public static (double Lat, double Lon) LatLonToMercator(double lat, double lon)
    {
        const double R = 6378137.0;
        double mx = lon * Math.PI / 180.0 * R;
        double my = Math.Log(Math.Tan(Math.PI / 4.0 + lat * Math.PI / 360.0)) * R;
        return (mx, my);
    }

    // MISSION_ITEM_INT x/y (degE7) -> Stride world position (metres, 1 unit = 1 m)
    public static Vector3 MavlinkToStride(int latE7, int lonE7, float altMeters = 0f)
    {
        double lat = latE7 / 1e7;
        double lon = lonE7 / 1e7;

        double north = (lat - SceneOriginGPS.Lat) * MetersPerDeg.Lat;
        double east = (lon - SceneOriginGPS.Lon) * MetersPerDeg.Lon;

        return new Vector3((float)east, altMeters, (float)(-north));
    }

    // Reverse: Stride position -> degE7 for sending back
    public static (int latE7, int lonE7, int altMm, int relAltMm) StrideToMavlink(Vector3 p)
    {
        double north = -p.Z;
        double east = p.X;

        double lat = SceneOriginGPS.Lat + north / MetersPerDeg.Lat;
        double lon = SceneOriginGPS.Lon + east / MetersPerDeg.Lon;

        int altMm = (int)Math.Round((SceneOriginGPS3.Alt + p.Y) * 1000.0); // AMSL, mm
        int relAltMm = (int)Math.Round(p.Y * 1000.0);                      // above home, mm

        return (
            (int)Math.Round(lat * 1e7),
            (int)Math.Round(lon * 1e7),
            altMm,
            relAltMm
        );
    }
}
