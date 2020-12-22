package frc.sim;

import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.util.Units;
import frc.Constants;
import frc.UnitUtils;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;

class SwerveModuleModel{

    PWMSim wheelMotorCtrl;
    PWMSim azmthMotorCtrl;

    SimQuadratureEncoder wheelMotorEncoder;
    SimQuadratureEncoder angleMotorEncoder;

    double curLinearSpeed_mps = 0; //Positive = in curAngle_deg, Negative = opposite of curAngle_deg
    Rotation2d curAzmthAngle = Rotation2d.fromDegrees(0); //0 = toward front, 90 = toward left, 180 = toward back, 270 = toward right

    MotorGearboxWheelSim wheelMotor;
    SimpleMotorWithMassModel azmthMotor;

    final double WHEEL_GEAR_RATIO = 6.1;
    final double AZMTH_GEAR_RATIO = 150.0;

    final double AZMTH_EFFECTIVE_MOI = 0.01;

    Pose2d prevModulePose = null;
    Pose2d curModulePose  = null;



    public SwerveModuleModel(int wheelMotorIdx, int azmthMotorIdx, int wheelEncIdx, int azmthEncIdx){
        wheelMotorCtrl = new PWMSim(wheelMotorIdx);
        azmthMotorCtrl = new PWMSim(azmthMotorIdx);

        //wheelMotor = new SimpleMotorWithMassModel(DCMotor.getNEO(1), WHEEL_GEAR_RATIO, WHEEL_EFFECTIVE_MOI);
        wheelMotor = new MotorGearboxWheelSim(DCMotor.getNEO(1), WHEEL_GEAR_RATIO, Units.inchesToMeters(Constants.WHEEL_RADIUS_IN * 2), 0.01);
        azmthMotor = new SimpleMotorWithMassModel(DCMotor.getVex775Pro(1), AZMTH_GEAR_RATIO, AZMTH_EFFECTIVE_MOI);

        wheelMotorEncoder = new SimQuadratureEncoder(wheelEncIdx, wheelEncIdx + 1, Constants.ENC_PULSE_PER_REV, Constants.AZMTH_ENC_MODULE_REVS_PER_COUNT);
        angleMotorEncoder = new SimQuadratureEncoder(azmthEncIdx, azmthEncIdx + 1, Constants.ENC_PULSE_PER_REV, Constants.WHEEL_ENC_WHEEL_REVS_PER_COUNT);
    }

    public void update(boolean isDisabled, double batteryVoltage){
        double wheelCmd = 0;
        double azmthCmd = 0;

        if(!isDisabled){
            wheelCmd = wheelMotorCtrl.getSpeed();
            azmthCmd = azmthMotorCtrl.getSpeed();
        }

        motionModel(wheelCmd, azmthCmd, batteryVoltage); 

        wheelMotorEncoder.setShaftPositionRev(wheelMotor.getPosition_Rev());
        angleMotorEncoder.setShaftPositionRev(azmthMotor.getPosition_Rev());
    }

    public void motionModel(double wheelCmd, double angleCmd, double batteryVoltage_v){

        //Calculate motion along the azimuth angle
        double xVel = (curModulePose.getTranslation().getX() - prevModulePose.getTranslation().getX())/Constants.SAMPLE_RATE_SEC;
        double yVel = (curModulePose.getTranslation().getY() - prevModulePose.getTranslation().getY())/Constants.SAMPLE_RATE_SEC;
        Vector2d moduleTranslationVec= new Vector2d(xVel,yVel);

        Vector2d azimuthUnitVec = new Vector2d(1,0);
        azimuthUnitVec.rotate(curAzmthAngle.getDegrees());

        double velocityAlongAzimuth = moduleTranslationVec.dot(azimuthUnitVec);

        wheelMotor.update(velocityAlongAzimuth, batteryVoltage_v, wheelCmd);

        azmthMotor.update(batteryVoltage_v, angleCmd);

        curAzmthAngle = Rotation2d.fromDegrees(azmthMotor.getPosition_Rev() * 360);
    }

    public double getCurrentDraw_A(){
        return wheelMotor.getCurrent_A() + azmthMotor.getCurrent_A();
    }

    public Rotation2d getCurAzmthAngle(){
        return curAzmthAngle;
    }
    
    /** Returns the current force in the Robot Frame reference frame */
    public Force2d getWheelMotiveForce(){
        return new Force2d(wheelMotor.getGroundForce_N(), curAzmthAngle);
    }

    /** Set the motion of each module in the Robot reference frame */
    public void setModulePose(Pose2d curPos){
        //Handle init'ing module position history to current on first pass
        if(prevModulePose == null){
            prevModulePose = curPos;
        } else {
            prevModulePose = curModulePose;
        }

        curModulePose = curPos;
    }

}