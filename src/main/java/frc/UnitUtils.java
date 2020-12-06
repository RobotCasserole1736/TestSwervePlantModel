package frc;

import edu.wpi.first.wpilibj.util.Units;

public class UnitUtils{

    public static double DtRPMtoMPerSec(double rot_spd_in){ return rot_spd_in * (1.0/60.0) * (2.0 * Math.PI * Units.inchesToMeters(Constants.WHEEL_RADIUS_IN)); }
    public static double DtMPerSectoRPM(double lin_spd_in){ return lin_spd_in / (1.0/60.0) / (2.0 * Math.PI * Units.inchesToMeters(Constants.WHEEL_RADIUS_IN)); }
    public static double RPMtoDegPerSec(double rpmIn){ return rpmIn * 60.0 / 360.0; }


}