package frc;

import edu.wpi.first.wpilibj.util.Units;

public class Constants {

    /******* Configured *******/
    
    static public final double SAMPLE_RATE_SEC = 0.02;

    static public final double WHEEL_BASE_HALF_WIDTH_M = Units.feetToMeters(1.0);

    static public final double WHEEL_RADIUS_IN = 6.0/2.0; //six inch diameter wheels

    static public final int ENC_PULSE_PER_REV = 1024;

    static public final int WHEEL_ENC_COUNTS_PER_WHEEL_REV = ENC_PULSE_PER_REV;  //Assume 1-1 gearing for now
    static public final int AZMTH_ENC_COUNTS_PER_MODULE_REV = ENC_PULSE_PER_REV; //Assume 1-1 gearing for now

    static public final double ROBOT_MASS_kg = lbsToKg(140);

    /******* Derived *******/
    
    static public final double WHEEL_ENC_WHEEL_REVS_PER_COUNT  = 1.0/((double)(Constants.WHEEL_ENC_COUNTS_PER_WHEEL_REV));
    static public final double AZMTH_ENC_MODULE_REVS_PER_COUNT = 1.0/((double)(Constants.AZMTH_ENC_COUNTS_PER_MODULE_REV));

    static public final double ROBOT_MOI_KGM2 = 1.0/12.0 * ROBOT_MASS_kg * Math.pow((WHEEL_BASE_HALF_WIDTH_M*2.2),2) * 2; //square slab slightly bigger than wheelbase with axis through center

    static double lbsToKg(double lbs_in){
        return 0.4535924 * lbs_in;
    }

}