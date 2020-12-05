package frc.sim;

import frc.patch.Field2d; //TODO: Pick up actual wpi version of this after bugcixes completed.
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.DataServer.Signal;

class DrivetrainModel {

    // Locations for the swerve drive modules relative to the robot center.
    Translation2d m_FLModuleTrans = new Translation2d( SimConstants.WHEEL_BASE_HALF_WIDTH_M,  SimConstants.WHEEL_BASE_HALF_WIDTH_M);
    Translation2d m_FRModuleTrans = new Translation2d( SimConstants.WHEEL_BASE_HALF_WIDTH_M, -SimConstants.WHEEL_BASE_HALF_WIDTH_M);
    Translation2d m_BLModuleTrans = new Translation2d(-SimConstants.WHEEL_BASE_HALF_WIDTH_M,  SimConstants.WHEEL_BASE_HALF_WIDTH_M);
    Translation2d m_BRModuleTrans = new Translation2d(-SimConstants.WHEEL_BASE_HALF_WIDTH_M, -SimConstants.WHEEL_BASE_HALF_WIDTH_M);

    // Creating my kinematics object using the module locations
    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    m_FLModuleTrans, m_FRModuleTrans, m_BLModuleTrans, m_BRModuleTrans
    );

    SwerveDriveOdometry m_odometry;

    SwerveModuleModel FLModule;
    SwerveModuleModel FRModule;
    SwerveModuleModel BLModule;
    SwerveModuleModel BRModule;


    Signal xPosFtSig;
    Signal yPosFtSig;
    Signal tRotDegSig;

    Signal xPosDesFtSig;
    Signal yPosDesFtSig;
    Signal tRotDesDegSig;

    Field2d field;
    Pose2d dtPoseForTelemetry;

    final Pose2d START_POSE = new Pose2d(Utils.ftToM(13.0), Utils.ftToM(5.0), Rotation2d.fromDegrees(0));

    public DrivetrainModel(){

        FLModule = new SwerveModuleModel(0, 1);
        FRModule = new SwerveModuleModel(2, 3);
        BLModule = new SwerveModuleModel(4, 5);
        BRModule = new SwerveModuleModel(6, 7);

        m_odometry = new SwerveDriveOdometry(m_kinematics, Rotation2d.fromDegrees(0.0), START_POSE);

        xPosFtSig     = new Signal("botActPoseX", "ft");
        yPosFtSig     = new Signal("botActPoseY", "ft");
        tRotDegSig    = new Signal("botActPoseT", "deg");
        xPosDesFtSig  = new Signal("botDesPoseX", "ft");
        yPosDesFtSig  = new Signal("botDesPoseY", "ft");
        tRotDesDegSig = new Signal("botDesPoseT", "deg");

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

    public void updateTelemetry(double time){
        xPosFtSig.addSample(time,  Utils.mToFt(dtPoseForTelemetry.getTranslation().getX()));
        yPosFtSig.addSample(time,  Utils.mToFt(dtPoseForTelemetry.getTranslation().getY()));
        tRotDegSig.addSample(time, dtPoseForTelemetry.getRotation().getDegrees());
        xPosDesFtSig.addSample(time,  0);
        yPosDesFtSig.addSample(time,  0);
        tRotDesDegSig.addSample(time, 0);
    }

}