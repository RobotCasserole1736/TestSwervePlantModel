package frc.sim;

class SimConstants {
    
    static final double SAMPLE_RATE_SEC = 0.02;

    static final double WHEEL_BASE_HALF_WIDTH_M = Utils.ftToM(1.0);


    static final double DT_MAX_SPEED_FT_PER_SEC = 16.0;
    static final double DT_ANGLE_MAX_SPEED_RPM = 90;
    static final double WHEEL_RADIUS_FT = 6.0/2.0/12.0; //six inch diameter wheels

    static final double WHEEL_ENC_COUNTS_PER_WHEEL_REV = 1024;
    static final double ANGLE_ENC_COUNTS_PER_MODULE_REV = 1024;


}