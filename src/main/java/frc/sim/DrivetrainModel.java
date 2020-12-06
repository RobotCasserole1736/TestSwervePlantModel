package frc.sim;

import frc.patch.Field2d; //TODO: Pick up actual wpi version of this after bugcixes completed.
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import frc.Constants;
import frc.lib.DataServer.Signal;

class DrivetrainModel {

    // Locations for the swerve drive modules relative to the robot center.
    Translation2d m_FLModuleTrans = new Translation2d( Constants.WHEEL_BASE_HALF_WIDTH_M,  Constants.WHEEL_BASE_HALF_WIDTH_M);
    Translation2d m_FRModuleTrans = new Translation2d( Constants.WHEEL_BASE_HALF_WIDTH_M, -Constants.WHEEL_BASE_HALF_WIDTH_M);
    Translation2d m_BLModuleTrans = new Translation2d(-Constants.WHEEL_BASE_HALF_WIDTH_M,  Constants.WHEEL_BASE_HALF_WIDTH_M);
    Translation2d m_BRModuleTrans = new Translation2d(-Constants.WHEEL_BASE_HALF_WIDTH_M, -Constants.WHEEL_BASE_HALF_WIDTH_M);

    // Creating my kinematics object using the module locations
    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    m_FLModuleTrans, m_FRModuleTrans, m_BLModuleTrans, m_BRModuleTrans
    );

    SwerveDriveOdometry m_odometry;

    SwerveModuleModel FLModule;
    SwerveModuleModel FRModule;
    SwerveModuleModel BLModule;
    SwerveModuleModel BRModule;

    ADXRS450_GyroSim gyroSim;

    Signal xPosActFtSig;
    Signal yPosActFtSig;
    Signal tRotActDegSig;

    Field2d field;
    Pose2d dtPoseForTelemetry;

    final Pose2d START_POSE = new Pose2d(Units.feetToMeters(13.0), Units.feetToMeters(5.0), Rotation2d.fromDegrees(0));

    public DrivetrainModel(){

        FLModule = new SwerveModuleModel(0, 1, 0,  2);
        FRModule = new SwerveModuleModel(2, 3, 4,  6);
        BLModule = new SwerveModuleModel(4, 5, 8,  10);
        BRModule = new SwerveModuleModel(6, 7, 12, 14);

        m_odometry = new SwerveDriveOdometry(m_kinematics, Rotation2d.fromDegrees(0.0), START_POSE);

        field = new Field2d();
        field.setRobotPose(START_POSE);
        SmartDashboard.putData("field", field);

        dtPoseForTelemetry = new Pose2d();
    }

    public void modelReset(){
        field.setRobotPose(START_POSE);
    }

    public void update(boolean isDisabled){

        Pose2d dtPos = field.getRobotPose();

        FLModule.update(isDisabled);
        FRModule.update(isDisabled);
        BLModule.update(isDisabled);
        BRModule.update(isDisabled);

        m_odometry.resetPosition(dtPos, Rotation2d.fromDegrees(0.0));

        m_odometry.update(dtPos.getRotation(), FLModule.getState(), FRModule.getState(), BLModule.getState(), BRModule.getState());

        dtPos = m_odometry.getPoseMeters();

        field.setRobotPose(dtPos);

        dtPoseForTelemetry = dtPos;
    }

}