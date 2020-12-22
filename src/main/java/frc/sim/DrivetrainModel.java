package frc.sim;

import frc.patch.Field2d; //TODO: Pick up actual wpi version of this after bugcixes completed.
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
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
    
    Vector2d m_FLModuleVec = new Vector2d(m_FLModuleTrans.getX(), m_FLModuleTrans.getY());
    Vector2d m_FRModuleVec = new Vector2d(m_FRModuleTrans.getX(), m_FRModuleTrans.getY());
    Vector2d m_BLModuleVec = new Vector2d(m_BLModuleTrans.getX(), m_BLModuleTrans.getY());
    Vector2d m_BRModuleVec = new Vector2d(m_BRModuleTrans.getX(), m_BRModuleTrans.getY());

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

    Vector2d accel_prev = new Vector2d();
    Vector2d vel_prev   = new Vector2d();

    double rotAccel_prev = 0;
    double rotVel_prev   = 0;

    final Pose2d START_POSE = new Pose2d(Units.feetToMeters(13.0), Units.feetToMeters(5.0), Rotation2d.fromDegrees(0));

    public DrivetrainModel(){

        FLModule = new SwerveModuleModel(0, 1, 0,  2);
        FRModule = new SwerveModuleModel(2, 3, 4,  6);
        BLModule = new SwerveModuleModel(4, 5, 8,  10);
        BRModule = new SwerveModuleModel(6, 7, 12, 14);

        m_odometry = new SwerveDriveOdometry(m_kinematics, Rotation2d.fromDegrees(0.0), START_POSE);

        gyroSim = new ADXRS450_GyroSim( new ADXRS450_Gyro()); //Use default gyro port and some new instance to not require tie to user code.

        field = new Field2d();
        field.setRobotPose(START_POSE);
        SmartDashboard.putData("field", field);

        dtPoseForTelemetry = new Pose2d();
    }

    public void modelReset(){
        field.setRobotPose(START_POSE);
    }

    public void update(boolean isDisabled, double batteryVoltage){

        Pose2d startPos = field.getRobotPose();

        FLModule.setModulePose(startPos.transformBy(new Transform2d(m_FLModuleTrans, new Rotation2d())));
        FRModule.setModulePose(startPos.transformBy(new Transform2d(m_FRModuleTrans, new Rotation2d())));
        BLModule.setModulePose(startPos.transformBy(new Transform2d(m_BLModuleTrans, new Rotation2d())));
        BRModule.setModulePose(startPos.transformBy(new Transform2d(m_BRModuleTrans, new Rotation2d())));

        FLModule.update(isDisabled, batteryVoltage);
        FRModule.update(isDisabled, batteryVoltage);
        BLModule.update(isDisabled, batteryVoltage);
        BRModule.update(isDisabled, batteryVoltage);

        //Sum of Forces
        Force2d netForce = new Force2d();
        netForce = netForce.plus(FLModule.getWheelMotiveForce())
                           .plus(FRModule.getWheelMotiveForce())
                           .plus(BLModule.getWheelMotiveForce())
                           .plus(BRModule.getWheelMotiveForce());

        //Sum of Torques
        Vector2d FLLeverArmVec = m_FLModuleVec;
        Vector2d FRLeverArmVec = m_FRModuleVec;
        Vector2d BLLeverArmVec = m_BLModuleVec;
        Vector2d BRLeverArmVec = m_BRModuleVec;
        FLLeverArmVec.rotate(90);
        FRLeverArmVec.rotate(90);
        BLLeverArmVec.rotate(90);
        BRLeverArmVec.rotate(90);
        double netTorque =  FLModule.getWheelMotiveForce().vec.dot(FLLeverArmVec)
                         +  FRModule.getWheelMotiveForce().vec.dot(FRLeverArmVec)
                         +  BLModule.getWheelMotiveForce().vec.dot(BLLeverArmVec)
                         +  BRModule.getWheelMotiveForce().vec.dot(BRLeverArmVec);

        //a = F/m
        Vector2d accel = netForce.times(1/Constants.ROBOT_MASS_kg).vec;

        Vector2d velocity = new Vector2d( vel_prev.x + (accel.x + accel_prev.x)/2 * Constants.SAMPLE_RATE_SEC, //Trapezoidal integration
                                          vel_prev.y + (accel.y + accel_prev.y)/2 * Constants.SAMPLE_RATE_SEC);

        Translation2d posChange = new Translation2d( (velocity.x + vel_prev.x)/2 * Constants.SAMPLE_RATE_SEC, //Trapezoidal integration
                                                     (velocity.y + vel_prev.y)/2 * Constants.SAMPLE_RATE_SEC);
        
        vel_prev = velocity;
        accel_prev = accel;
        
        //alpha = T/I
        double rotAccel = netTorque / Constants.ROBOT_MOI_KGM2;
        double rotVel = rotVel_prev + (rotAccel + rotAccel_prev)/2 * Constants.SAMPLE_RATE_SEC;
        double rotPosChange = (rotVel + rotVel_prev)/2 * Constants.SAMPLE_RATE_SEC;

        rotVel_prev = rotVel;
        rotAccel_prev = rotAccel;


        Pose2d endPose = startPos.transformBy( new Transform2d(posChange, new Rotation2d(rotPosChange)));

        double curGyroAngle = endPose.getRotation().getDegrees();
        double prevGyroAngle = startPos.getRotation().getDegrees();
        double gyroRate = (curGyroAngle - prevGyroAngle)/Constants.SAMPLE_RATE_SEC;

        gyroSim.setAngle( -1.0 * curGyroAngle);
        gyroSim.setRate(  -1.0 * gyroRate);

        field.setRobotPose(endPose);

        dtPoseForTelemetry = endPose;
    }

    public double getCurrentDraw(){
        return FLModule.getCurrentDraw_A() + 
               FRModule.getCurrentDraw_A() + 
               BLModule.getCurrentDraw_A() + 
               BRModule.getCurrentDraw_A();
    }

}