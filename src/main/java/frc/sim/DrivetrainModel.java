package frc.sim;

import frc.patch.Field2d; //TODO: Pick up actual wpi version of this after bugcixes completed.
import frc.sim.physics.Force2d;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.geometry.Twist2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import frc.Constants;
import frc.lib.DataServer.Signal;

class DrivetrainModel {

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
    double   rotAccel_prev = 0;
    double   rotVel_prev   = 0;

    final Pose2d START_POSE = new Pose2d(Units.feetToMeters(13.0), Units.feetToMeters(5.0), Rotation2d.fromDegrees(45));

    public DrivetrainModel(){

        FLModule = new SwerveModuleModel(0, 1, 0,  2);
        FRModule = new SwerveModuleModel(2, 3, 4,  6);
        BLModule = new SwerveModuleModel(4, 5, 8,  10);
        BRModule = new SwerveModuleModel(6, 7, 12, 14);

        m_odometry = new SwerveDriveOdometry(Constants.m_kinematics, Rotation2d.fromDegrees(0.0), START_POSE);

        gyroSim = new ADXRS450_GyroSim( new ADXRS450_Gyro()); //Use default gyro port and some new instance to not require tie to user code.

        field = new Field2d();
        field.setRobotPose(START_POSE);
        SmartDashboard.putData("field", field);

        dtPoseForTelemetry = new Pose2d();
    }

    public void modelReset(){
        field.setRobotPose(START_POSE);
        accel_prev = new Vector2d();
        vel_prev   = new Vector2d();
        rotAccel_prev = 0;
        rotVel_prev   = 0;
    }

    public void update(boolean isDisabled, double batteryVoltage){

        Pose2d startPos = field.getRobotPose();

        FLModule.setModulePose(startPos.transformBy(new Transform2d(Constants.m_FLModuleTrans, new Rotation2d())));
        FRModule.setModulePose(startPos.transformBy(new Transform2d(Constants.m_FRModuleTrans, new Rotation2d())));
        BLModule.setModulePose(startPos.transformBy(new Transform2d(Constants.m_BLModuleTrans, new Rotation2d())));
        BRModule.setModulePose(startPos.transformBy(new Transform2d(Constants.m_BRModuleTrans, new Rotation2d())));

        FLModule.update(isDisabled, batteryVoltage);
        FRModule.update(isDisabled, batteryVoltage);
        BLModule.update(isDisabled, batteryVoltage);
        BRModule.update(isDisabled, batteryVoltage);

        //Sum of Forces

        double netTorque = 0;

        //External Forces on frame
        Force2d extForcePerModule = new Force2d();

        if(RobotController.getUserButton()){
            extForcePerModule = extForcePerModule.plus(new Force2d(0, 700));
            //netTorque += 10;
        }
        extForcePerModule = applyWallCollisions(extForcePerModule, startPos);

        //Force on frame from wheels
        Force2d netPreFricForceFL = extForcePerModule.plus(FLModule.getWheelMotiveForce());
        Force2d netPreFricForceFR = extForcePerModule.plus(FRModule.getWheelMotiveForce());
        Force2d netPreFricForceBL = extForcePerModule.plus(BLModule.getWheelMotiveForce());
        Force2d netPreFricForceBR = extForcePerModule.plus(BRModule.getWheelMotiveForce());
        Force2d netPreFricForce = netPreFricForceFL.plus(netPreFricForceFR).plus(netPreFricForceBL).plus(netPreFricForceBR);

        //Reactive friction forces
        Force2d netFricForceFL = FLModule.getCrossTreadFrictionalForce();
        Force2d netFricForceFR = FRModule.getCrossTreadFrictionalForce();
        Force2d netFricForceBL = BLModule.getCrossTreadFrictionalForce();
        Force2d netFricForceBR = BRModule.getCrossTreadFrictionalForce();

        Force2d netForceFL = netPreFricForceFL.plus(netFricForceFL);
        Force2d netForceFR = netPreFricForceFR.plus(netFricForceFR);
        Force2d netForceBL = netPreFricForceBL.plus(netFricForceBL);
        Force2d netForceBR = netPreFricForceBR.plus(netFricForceBR);

        Force2d netForce = netForceFL.plus(netForceFR).plus(netForceBL).plus(netForceBR);

        //Sum of Torques
        Vector2d FLLeverArmVec = Constants.m_FLModuleVec;
        Vector2d FRLeverArmVec = Constants.m_FRModuleVec;
        Vector2d BLLeverArmVec = Constants.m_BLModuleVec;
        Vector2d BRLeverArmVec = Constants.m_BRModuleVec;
        FLLeverArmVec.rotate(90);
        FRLeverArmVec.rotate(90);
        BLLeverArmVec.rotate(90);
        BRLeverArmVec.rotate(90);
        netTorque +=  netForceFL.vec.dot(FLLeverArmVec);
        netTorque +=  netForceFR.vec.dot(FRLeverArmVec);
        netTorque +=  netForceBL.vec.dot(BLLeverArmVec);
        netTorque +=  netForceBR.vec.dot(BRLeverArmVec);


        //a = F/m
        Vector2d accel = netForce.times(1/Constants.ROBOT_MASS_kg).vec;

        Vector2d velocity = new Vector2d( vel_prev.x + (accel.x + accel_prev.x)/2 * Constants.SIM_SAMPLE_RATE_SEC, //Trapezoidal integration
                                          vel_prev.y + (accel.y + accel_prev.y)/2 * Constants.SIM_SAMPLE_RATE_SEC);

        Translation2d posChange = new Translation2d( (velocity.x + vel_prev.x)/2 * Constants.SIM_SAMPLE_RATE_SEC, //Trapezoidal integration
                                                     (velocity.y + vel_prev.y)/2 * Constants.SIM_SAMPLE_RATE_SEC);
        
        vel_prev = velocity;
        accel_prev = accel;
        
        //alpha = T/I
        double rotAccel = netTorque / Constants.ROBOT_MOI_KGM2;
        double rotVel = rotVel_prev + (rotAccel + rotAccel_prev)/2 * Constants.SIM_SAMPLE_RATE_SEC;
        double rotPosChange = (rotVel + rotVel_prev)/2 * Constants.SIM_SAMPLE_RATE_SEC;

        rotVel_prev = rotVel;
        rotAccel_prev = rotAccel;

        posChange = posChange.rotateBy(startPos.getRotation());

        Twist2d motionThisLoop = new Twist2d(posChange.getX(), posChange.getY(), rotPosChange);

        Pose2d endPose = startPos.exp(motionThisLoop);

        double curGyroAngle = endPose.getRotation().getDegrees();
        double prevGyroAngle = startPos.getRotation().getDegrees();
        double gyroRate = (curGyroAngle - prevGyroAngle)/Constants.SIM_SAMPLE_RATE_SEC;

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

    // Very rough approximation of bumpers wacking into a wall.
    // Assumes wall is a very peculiar form of squishy and sticky.
    public Force2d applyWallCollisions(Force2d netForce_in, Pose2d pos_in){
        final double WALL_PUSHY_FORCE_N = 300; 

        if(pos_in.getX() > Constants.MAX_ROBOT_TRANSLATION.getX()){
            //Too far in the positive X direction
            netForce_in = new Force2d(-WALL_PUSHY_FORCE_N, 0);
        }else if(pos_in.getX() < Constants.MIN_ROBOT_TRANSLATION.getX()){
            //Too far in the negative X direction
            netForce_in = new Force2d(WALL_PUSHY_FORCE_N, 0);
        }

        if(pos_in.getY() > Constants.MAX_ROBOT_TRANSLATION.getY()){
            //Too far in the positive Y direction
            netForce_in = new Force2d(0, -WALL_PUSHY_FORCE_N);
        }else if(pos_in.getY() < Constants.MIN_ROBOT_TRANSLATION.getY()){
            //Too far in the negative Y direction
            netForce_in = new Force2d(0, WALL_PUSHY_FORCE_N);
        }

        return netForce_in;
    }

}