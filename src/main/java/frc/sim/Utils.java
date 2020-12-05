package frc.sim;

class Utils{

    static double ftToM(double ft_in){ return ft_in * 0.3048; }
    static double mToFt(double m_in){ return m_in / 0.3048; }
    static double DtRPMtoMPerSec(double rot_spd_in){ return rot_spd_in * (1.0/60.0) * (2.0 * Math.PI * ftToM(SimConstants.WHEEL_RADIUS_FT)); }
    static double DtMPerSectoRPM(double lin_spd_in){ return lin_spd_in / (1.0/60.0) / (2.0 * Math.PI * ftToM(SimConstants.WHEEL_RADIUS_FT)); }
    static double RPMtoDegPerSec(double rpmIn){ return rpmIn * 60.0 / 360.0; }


}