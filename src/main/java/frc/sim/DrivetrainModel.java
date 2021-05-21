package frc.sim;

import frc.patch.Field2d; //TODO: Pick up actual wpi version of this after bugcixes completed.
import frc.sim.physics.Force2d;
import frc.sim.physics.Vector2d;
import frc.sim.physics.ForceAtPose2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.geometry.Twist2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Constants;
import frc.lib.DataServer.Signal;

class DrivetrainModel {

    SwerveModuleModel FLModule;
    SwerveModuleModel FRModule;
    SwerveModuleModel BLModule;
    SwerveModuleModel BRModule;

    SimGyroSensorModel gyro;
    SimVisionModel vision;

    Signal xPosActFtSig;
    Signal yPosActFtSig;
    Signal tRotActDegSig;

    Field2d field;
    Pose2d dtPoseForTelemetry;
    Pose2d endRobotRefFrame; //Previous sent pose, allows us to detect when the robot has been manually moved.

    Vector2d accel_prev = new Vector2d();
    Vector2d vel_prev   = new Vector2d();
    double   rotAccel_prev = 0;
    double   rotVel_prev   = 0;

    public DrivetrainModel(){

        FLModule = new SwerveModuleModel(Constants.FL_WHEEL_MOTOR_IDX,Constants.FL_AZMTH_MOTOR_IDX,Constants.FL_WHEEL_ENC_A_IDX,Constants.FL_AZMTH_ENC_A_IDX);
        FRModule = new SwerveModuleModel(Constants.FR_WHEEL_MOTOR_IDX,Constants.FR_AZMTH_MOTOR_IDX,Constants.FR_WHEEL_ENC_A_IDX,Constants.FR_AZMTH_ENC_A_IDX);
        BLModule = new SwerveModuleModel(Constants.BL_WHEEL_MOTOR_IDX,Constants.BL_AZMTH_MOTOR_IDX,Constants.BL_WHEEL_ENC_A_IDX,Constants.BL_AZMTH_ENC_A_IDX);
        BRModule = new SwerveModuleModel(Constants.BR_WHEEL_MOTOR_IDX,Constants.BR_AZMTH_MOTOR_IDX,Constants.BR_WHEEL_ENC_A_IDX,Constants.BR_AZMTH_ENC_A_IDX);

        gyro = new SimGyroSensorModel();
        vision = new SimVisionModel();

        field = new Field2d();
        field.setRobotPose(Constants.DFLT_START_POSE);
        SmartDashboard.putData("field", field);

        dtPoseForTelemetry = new Pose2d();
    }

    public void modelReset(Pose2d pose){
        field.setRobotPose(pose);
        accel_prev = new Vector2d();
        vel_prev   = new Vector2d();
        rotAccel_prev = 0;
        rotVel_prev   = 0;
        FLModule.reset(pose.transformBy(Constants.robotToFLModuleTrans));
        FRModule.reset(pose.transformBy(Constants.robotToFRModuleTrans));
        BLModule.reset(pose.transformBy(Constants.robotToBLModuleTrans));
        BRModule.reset(pose.transformBy(Constants.robotToBRModuleTrans));
        gyro.resetToPose(pose);
    }

    public void update(boolean isDisabled, double batteryVoltage){

        Pose2d fieldReferenceFrame = new Pose2d();// global origin
        Pose2d startRobotRefFrame = field.getRobotPose(); //orgin on and aligned to robot's present position in the field

        if(!startRobotRefFrame.equals(endRobotRefFrame)){
            //Robot has been moved manually in the Field2D widget. Reset the sim model.
            endRobotRefFrame = startRobotRefFrame;
            modelReset(startRobotRefFrame);
        }

        Transform2d fieldToRobotTrans = new Transform2d(fieldReferenceFrame, startRobotRefFrame);

        Pose2d FLModuleRefFrame = fieldReferenceFrame.transformBy(fieldToRobotTrans).transformBy(Constants.robotToFLModuleTrans);
        Pose2d FRModuleRefFrame = fieldReferenceFrame.transformBy(fieldToRobotTrans).transformBy(Constants.robotToFRModuleTrans);
        Pose2d BLModuleRefFrame = fieldReferenceFrame.transformBy(fieldToRobotTrans).transformBy(Constants.robotToBLModuleTrans);
        Pose2d BRModuleRefFrame = fieldReferenceFrame.transformBy(fieldToRobotTrans).transformBy(Constants.robotToBRModuleTrans);

        FLModule.setModulePose(FLModuleRefFrame);
        FRModule.setModulePose(FRModuleRefFrame);
        BLModule.setModulePose(BLModuleRefFrame);
        BRModule.setModulePose(BRModuleRefFrame);

        FLModule.update(isDisabled, batteryVoltage);
        FRModule.update(isDisabled, batteryVoltage);
        BLModule.update(isDisabled, batteryVoltage);
        BRModule.update(isDisabled, batteryVoltage);


        double netTorque = 0;

        //External Forces on chassis, in chassis reference frame.
        ForceAtPose2d sideKickForce = new ForceAtPose2d(new Force2d(0,0), startRobotRefFrame);
        if(RobotController.getUserButton()){
            sideKickForce.force = new Force2d(0, 700);
            //netTorque += 10;
        }

        //Force on frame from wheels
        ForceAtPose2d wheelMotiveForceFL = FLModule.getWheelMotiveForce();
        ForceAtPose2d wheelMotiveForceFR = FRModule.getWheelMotiveForce();
        ForceAtPose2d wheelMotiveForceBL = BLModule.getWheelMotiveForce();
        ForceAtPose2d wheelMotiveForceBR = BRModule.getWheelMotiveForce();

        Force2d preFricNetForce = new Force2d();
        preFricNetForce = preFricNetForce.plus(wheelMotiveForceFL.getForceInRefFrame(startRobotRefFrame));
        preFricNetForce = preFricNetForce.plus(wheelMotiveForceFR.getForceInRefFrame(startRobotRefFrame));
        preFricNetForce = preFricNetForce.plus(wheelMotiveForceBL.getForceInRefFrame(startRobotRefFrame));
        preFricNetForce = preFricNetForce.plus(wheelMotiveForceBR.getForceInRefFrame(startRobotRefFrame));

        ForceAtPose2d preFricNetForceRobotCenter = new ForceAtPose2d(preFricNetForce, startRobotRefFrame);

        //Reactive friction forces
        ForceAtPose2d netFricForceFL = FLModule.getCrossTreadFrictionalForce(preFricNetForceRobotCenter.getForceInRefFrame(FLModuleRefFrame));
        ForceAtPose2d netFricForceFR = FRModule.getCrossTreadFrictionalForce(preFricNetForceRobotCenter.getForceInRefFrame(FRModuleRefFrame));
        ForceAtPose2d netFricForceBL = BLModule.getCrossTreadFrictionalForce(preFricNetForceRobotCenter.getForceInRefFrame(BLModuleRefFrame));
        ForceAtPose2d netFricForceBR = BRModule.getCrossTreadFrictionalForce(preFricNetForceRobotCenter.getForceInRefFrame(BRModuleRefFrame));

        //Sum of Forces
        Force2d forceOnRobotCenter = new Force2d();

        forceOnRobotCenter = forceOnRobotCenter.plus(wheelMotiveForceFL.getForceInRefFrame(startRobotRefFrame));
        forceOnRobotCenter = forceOnRobotCenter.plus(wheelMotiveForceFR.getForceInRefFrame(startRobotRefFrame));
        forceOnRobotCenter = forceOnRobotCenter.plus(wheelMotiveForceBL.getForceInRefFrame(startRobotRefFrame));
        forceOnRobotCenter = forceOnRobotCenter.plus(wheelMotiveForceBR.getForceInRefFrame(startRobotRefFrame));

        forceOnRobotCenter = forceOnRobotCenter.plus(netFricForceFL.getForceInRefFrame(startRobotRefFrame));
        forceOnRobotCenter = forceOnRobotCenter.plus(netFricForceFR.getForceInRefFrame(startRobotRefFrame));
        forceOnRobotCenter = forceOnRobotCenter.plus(netFricForceBL.getForceInRefFrame(startRobotRefFrame));
        forceOnRobotCenter = forceOnRobotCenter.plus(netFricForceBR.getForceInRefFrame(startRobotRefFrame));

        forceOnRobotCenter = forceOnRobotCenter.plus(sideKickForce.getForceInRefFrame(startRobotRefFrame));
        
        ForceAtPose2d netForce = new ForceAtPose2d(forceOnRobotCenter, startRobotRefFrame);


        //Sum of Torques
        netTorque +=  wheelMotiveForceFL.getTorque(startRobotRefFrame) + netFricForceFL.getTorque(startRobotRefFrame);
        netTorque +=  wheelMotiveForceFR.getTorque(startRobotRefFrame) + netFricForceFR.getTorque(startRobotRefFrame);
        netTorque +=  wheelMotiveForceBL.getTorque(startRobotRefFrame) + netFricForceBL.getTorque(startRobotRefFrame);
        netTorque +=  wheelMotiveForceBR.getTorque(startRobotRefFrame) + netFricForceBR.getTorque(startRobotRefFrame);


        Force2d robotForceInFieldRefFrame = netForce.getForceInRefFrame(fieldReferenceFrame).plus(getWallCollisionForce(startRobotRefFrame));


        //a = F/m in field frame
        Vector2d accel = robotForceInFieldRefFrame.times(1/Constants.ROBOT_MASS_kg).vec;

        Vector2d velocity = new Vector2d( vel_prev.x + (accel.x + accel_prev.x)/2 * Constants.SIM_SAMPLE_RATE_SEC, //Trapezoidal integration
                                          vel_prev.y + (accel.y + accel_prev.y)/2 * Constants.SIM_SAMPLE_RATE_SEC);

        Translation2d posChange = new Translation2d( (velocity.x + vel_prev.x)/2 * Constants.SIM_SAMPLE_RATE_SEC, //Trapezoidal integration
                                                     (velocity.y + vel_prev.y)/2 * Constants.SIM_SAMPLE_RATE_SEC);
        
        vel_prev = velocity;
        accel_prev = accel;
        
        //alpha = T/I in field frame
        double rotAccel = netTorque / Constants.ROBOT_MOI_KGM2;
        double rotVel = rotVel_prev + (rotAccel + rotAccel_prev)/2 * Constants.SIM_SAMPLE_RATE_SEC;
        double rotPosChange = (rotVel + rotVel_prev)/2 * Constants.SIM_SAMPLE_RATE_SEC;

        rotVel_prev = rotVel;
        rotAccel_prev = rotAccel;

        posChange = posChange.rotateBy(startRobotRefFrame.getRotation().unaryMinus()); //Twist needs to be relative to robot reference frame

        Twist2d motionThisLoop = new Twist2d(posChange.getX(), posChange.getY(), rotPosChange);
        
        endRobotRefFrame = startRobotRefFrame.exp(motionThisLoop);


        field.setRobotPose(endRobotRefFrame);

        gyro.update(endRobotRefFrame, startRobotRefFrame);
        vision.update(endRobotRefFrame);

        dtPoseForTelemetry = endRobotRefFrame;
    }

    public double getCurrentDraw(){
        return FLModule.getCurrentDraw_A() + 
               FRModule.getCurrentDraw_A() + 
               BLModule.getCurrentDraw_A() + 
               BRModule.getCurrentDraw_A();
    }

    // Very rough approximation of bumpers wacking into a wall.
    // Assumes wall is a very peculiar form of squishy and sticky.
    public Force2d getWallCollisionForce(Pose2d pos_in){
        final double WALL_PUSHY_FORCE_N = 5000; 

        Force2d netForce_in = new Force2d();

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