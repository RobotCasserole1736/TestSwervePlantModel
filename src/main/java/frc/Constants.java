package frc;

import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.util.Units;

public class Constants {

    /******* Configured *******/
    
    static public final double SIM_SAMPLE_RATE_SEC = 0.001;
    static public final double CTRLS_SAMPLE_RATE_SEC = 0.02;

    static public final double WHEEL_BASE_HALF_WIDTH_M = Units.feetToMeters(1.0);

    static public final double WHEEL_RADIUS_IN = 6.0/2.0; //six inch diameter wheels

    static public final int ENC_PULSE_PER_REV = 1024;

    static public final int WHEEL_ENC_COUNTS_PER_WHEEL_REV = ENC_PULSE_PER_REV;  //Assume 1-1 gearing for now
    static public final int AZMTH_ENC_COUNTS_PER_MODULE_REV = ENC_PULSE_PER_REV; //Assume 1-1 gearing for now

    static public final double ROBOT_MASS_kg = UnitUtils.lbsToKg(140);

    static public final double FIELD_WIDTH_M = Units.feetToMeters(27.0);
    static public final double FIELD_LENGTH_M = Units.feetToMeters(54.0);

    /******* Derived *******/
    
    static public final double WHEEL_ENC_WHEEL_REVS_PER_COUNT  = 1.0/((double)(Constants.WHEEL_ENC_COUNTS_PER_WHEEL_REV));
    static public final double AZMTH_ENC_MODULE_REVS_PER_COUNT = 1.0/((double)(Constants.AZMTH_ENC_COUNTS_PER_MODULE_REV));

    static public final double ROBOT_MOI_KGM2 = 1.0/12.0 * ROBOT_MASS_kg * Math.pow((WHEEL_BASE_HALF_WIDTH_M*2.2),2) * 2; //square slab slightly bigger than wheelbase with axis through center

    static public final Translation2d MAX_ROBOT_TRANSLATION = new Translation2d(FIELD_LENGTH_M, FIELD_WIDTH_M);
    static public final Translation2d MIN_ROBOT_TRANSLATION = new Translation2d(0.0,0.0);

    // Locations for the swerve drive modules relative to the robot center.
    static public final Translation2d m_FLModuleTrans = new Translation2d( Constants.WHEEL_BASE_HALF_WIDTH_M,  Constants.WHEEL_BASE_HALF_WIDTH_M);
    static public final Translation2d m_FRModuleTrans = new Translation2d( Constants.WHEEL_BASE_HALF_WIDTH_M, -Constants.WHEEL_BASE_HALF_WIDTH_M);
    static public final Translation2d m_BLModuleTrans = new Translation2d(-Constants.WHEEL_BASE_HALF_WIDTH_M,  Constants.WHEEL_BASE_HALF_WIDTH_M);
    static public final Translation2d m_BRModuleTrans = new Translation2d(-Constants.WHEEL_BASE_HALF_WIDTH_M, -Constants.WHEEL_BASE_HALF_WIDTH_M);
    
    static public final Vector2d m_FLModuleVec = new Vector2d(m_FLModuleTrans.getX(), m_FLModuleTrans.getY());
    static public final Vector2d m_FRModuleVec = new Vector2d(m_FRModuleTrans.getX(), m_FRModuleTrans.getY());
    static public final Vector2d m_BLModuleVec = new Vector2d(m_BLModuleTrans.getX(), m_BLModuleTrans.getY());
    static public final Vector2d m_BRModuleVec = new Vector2d(m_BRModuleTrans.getX(), m_BRModuleTrans.getY());

    // Creating my kinematics object using the module locations
    static public final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        m_FLModuleTrans, m_FRModuleTrans, m_BLModuleTrans, m_BRModuleTrans
        );


}