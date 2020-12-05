package frc.sim;

import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;

class SwerveModuleModel{

    PWMSim wheelMotorCtrl;
    PWMSim angleMotorCtrl;

    EncoderSim wheelMotorEncoder;
    EncoderSim angleMotorEncoder;

    double curLinearSpeed_mps = 0; //Positive = in curAngle_deg, Negative = opposite of curAngle_deg
    Rotation2d curAngle = Rotation2d.fromDegrees(0); //0 = toward front, 90 = toward left, 180 = toward back, 270 = toward right

    SimpleMotorWithMassModel wheelMotor;
    SimpleMotorWithMassModel angleMotor;


    public SwerveModuleModel(int wheelIdx, int angleIdx){
        wheelMotorCtrl = new PWMSim(wheelIdx);
        angleMotorCtrl = new PWMSim(angleIdx);

        wheelMotor = new SimpleMotorWithMassModel(Utils.DtMPerSectoRPM(Utils.ftToM(SimConstants.DT_MAX_SPEED_FT_PER_SEC)), 0.1, 125);
        angleMotor = new SimpleMotorWithMassModel(SimConstants.DT_ANGLE_MAX_SPEED_RPM, 0.2, 30);

        //wheelMotorEncoder = EncoderSim.createForChannel(wheelIdx);
        //angleMotorEncoder = EncoderSim.createForChannel(angleIdx);
    }

    public void update(boolean isDisabled){
        double wheelCmd = 0;
        double angleCmd = 0;

        if(!isDisabled){
            wheelCmd = wheelMotorCtrl.getSpeed();
            angleCmd = angleMotorCtrl.getSpeed();
        }

        motionModel(wheelCmd, angleCmd, 12.5 ); //hardcode full battery voltage for now.

        //wheelMotorEncoder.setCount((int)Math.round(wheelMotor.getPosition_Rev() * SimConstants.WHEEL_ENC_COUNTS_PER_WHEEL_REV));
        //angleMotorEncoder.setCount((int)Math.round(angleMotor.getPosition_Rev() * SimConstants.ANGLE_ENC_COUNTS_PER_MODULE_REV));
    }

    public void motionModel(double wheelCmd, double angleCmd, double batteryVoltage_v){
        wheelMotor.update(batteryVoltage_v, wheelCmd, 0.0);
        angleMotor.update(batteryVoltage_v, angleCmd, 0.0);

        double curAngleDeg = angleMotor.getPosition_Rev() * 360;

        setMeasurements(wheelMotor.getSpeed_RPM(), curAngleDeg);
    }

    public void setMeasurements(double wheelSpeed_RPM, double angle_deg){
        curLinearSpeed_mps = Utils.DtRPMtoMPerSec(wheelSpeed_RPM);
        curAngle = Rotation2d.fromDegrees(angle_deg);
    }

    public double getCurrentDraw_A(){
        return wheelMotor.getCurrent_A() + angleMotor.getCurrent_A();
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(curLinearSpeed_mps, curAngle);
    } 

}