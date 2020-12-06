package frc;

import edu.wpi.first.wpilibj.util.Units;

public class Constants {

    /******* Configured *******/
    
    static public final double SAMPLE_RATE_SEC = 0.02;

    static public final double WHEEL_BASE_HALF_WIDTH_M = Units.feetToMeters(1.0);

    static public final double WHEEL_RADIUS_IN = 6.0/2.0; //six inch diameter wheels

    static public final int WHEEL_ENC_COUNTS_PER_WHEEL_REV = 1024;
    static public final int AZMTH_ENC_COUNTS_PER_MODULE_REV = 1024;

    /******* Derived *******/
    
    static public final double WHEEL_ENC_WHEEL_REVS_PER_COUNT  = 1.0/((double)(Constants.WHEEL_ENC_COUNTS_PER_WHEEL_REV));
    static public final double AZMTH_ENC_MODULE_REVS_PER_COUNT = 1.0/((double)(Constants.AZMTH_ENC_COUNTS_PER_MODULE_REV));

}