package frc;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.util.Units;

public class Constants {


    // Periodic code execution rates
    static public final double SIM_SAMPLE_RATE_SEC = 0.001;
    static public final double CTRLS_SAMPLE_RATE_SEC = 0.02;

    // Key robot physical dimensions and mass quantities.
    static public final double WHEEL_BASE_HALF_WIDTH_M = Units.feetToMeters(1.0);
    static public final double WHEEL_RADIUS_IN = 6.0/2.0; //six inch diameter wheels
    static public final double ROBOT_MASS_kg = UnitUtils.lbsToKg(140);
    static public final double ROBOT_MOI_KGM2 = 1.0/12.0 * ROBOT_MASS_kg * Math.pow((WHEEL_BASE_HALF_WIDTH_M*2.2),2) * 2; //square slab slightly bigger than wheelbase with axis through center

    // Assumes starting location of the robot
    // TODO make this more flexible and inherit from autonomous routines
    static public final Pose2d START_POSE = new Pose2d(Units.feetToMeters(24.0), Units.feetToMeters(10.0), Rotation2d.fromDegrees(180));

    // Sensor-related constants - lookup in datasheets or sensor configuration
    static public final String PHOTON_CAM_NAME = "MainCamera";
    static public final int ENC_PULSE_PER_REV = 1024;
    static public final int WHEEL_ENC_COUNTS_PER_WHEEL_REV = ENC_PULSE_PER_REV;  //Assume 1-1 gearing for now
    static public final int AZMTH_ENC_COUNTS_PER_MODULE_REV = ENC_PULSE_PER_REV; //Assume 1-1 gearing for now
    static public final double WHEEL_ENC_WHEEL_REVS_PER_COUNT  = 1.0/((double)(Constants.WHEEL_ENC_COUNTS_PER_WHEEL_REV));
    static public final double AZMTH_ENC_MODULE_REVS_PER_COUNT = 1.0/((double)(Constants.AZMTH_ENC_COUNTS_PER_MODULE_REV));

    //Field wall locations which limit drivetrain motion
    static public final double FIELD_WIDTH_M = Units.feetToMeters(27.0);
    static public final double FIELD_LENGTH_M = Units.feetToMeters(54.0);
    static public final Translation2d MAX_ROBOT_TRANSLATION = new Translation2d(FIELD_LENGTH_M, FIELD_WIDTH_M);
    static public final Translation2d MIN_ROBOT_TRANSLATION = new Translation2d(0.0,0.0);

    // Locations for the swerve drive modules relative to the robot center.
    static public final Translation2d m_FLModuleTrans = new Translation2d( Constants.WHEEL_BASE_HALF_WIDTH_M,  Constants.WHEEL_BASE_HALF_WIDTH_M);
    static public final Translation2d m_FRModuleTrans = new Translation2d( Constants.WHEEL_BASE_HALF_WIDTH_M, -Constants.WHEEL_BASE_HALF_WIDTH_M);
    static public final Translation2d m_BLModuleTrans = new Translation2d(-Constants.WHEEL_BASE_HALF_WIDTH_M,  Constants.WHEEL_BASE_HALF_WIDTH_M);
    static public final Translation2d m_BRModuleTrans = new Translation2d(-Constants.WHEEL_BASE_HALF_WIDTH_M, -Constants.WHEEL_BASE_HALF_WIDTH_M);

    static public final Transform2d robotToFLModuleTrans = new Transform2d(Constants.m_FLModuleTrans, new Rotation2d(0.0));
    static public final Transform2d robotToFRModuleTrans = new Transform2d(Constants.m_FRModuleTrans, new Rotation2d(0.0));
    static public final Transform2d robotToBLModuleTrans = new Transform2d(Constants.m_BLModuleTrans, new Rotation2d(0.0));
    static public final Transform2d robotToBRModuleTrans = new Transform2d(Constants.m_BRModuleTrans, new Rotation2d(0.0));

    static public final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        m_FLModuleTrans, m_FRModuleTrans, m_BLModuleTrans, m_BRModuleTrans
    );

    // Location of camera relative to robot center - currently front middle.
    static public final Transform2d robotToCameraTrans = new Transform2d(new Translation2d(WHEEL_BASE_HALF_WIDTH_M, 0), new Rotation2d(0.0));

    static public final Transform2d fieldToFarVisionTargetTrans   = new Transform2d( new Translation2d(FIELD_LENGTH_M, Units.feetToMeters(9.8541)), Rotation2d.fromDegrees(0));
    static public final Transform2d fieldToCloseVisionTargetTrans = new Transform2d( new Translation2d(Units.feetToMeters(0), Units.feetToMeters(17.14)), Rotation2d.fromDegrees(180));


}