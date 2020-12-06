package frc.sim;

import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.util.Units;
import frc.Constants;
import frc.UnitUtils;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;

class SwerveModuleModel{

    PWMSim wheelMotorCtrl;
    PWMSim azmthMotorCtrl;

    EncoderSim wheelMotorEncoder;
    EncoderSim angleMotorEncoder;

    double curLinearSpeed_mps = 0; //Positive = in curAngle_deg, Negative = opposite of curAngle_deg
    Rotation2d curAzmthAngle = Rotation2d.fromDegrees(0); //0 = toward front, 90 = toward left, 180 = toward back, 270 = toward right

    SimpleMotorWithMassModel wheelMotor;
    SimpleMotorWithMassModel azmthMotor;

    final double WHEEL_MAX_SPEED_FT_PER_SEC = 12.0;
    final double AZMTH_MAX_SPEED_RPM = 90.0;


    public SwerveModuleModel(int wheelMotorIdx, int azmthMotorIdx, int wheelEncIdx, int azmthEncIdx){
        wheelMotorCtrl = new PWMSim(wheelMotorIdx);
        azmthMotorCtrl = new PWMSim(azmthMotorIdx);

        wheelMotor = new SimpleMotorWithMassModel(UnitUtils.DtMPerSectoRPM(Units.feetToMeters(WHEEL_MAX_SPEED_FT_PER_SEC)), 0.1, 125);
        azmthMotor = new SimpleMotorWithMassModel(AZMTH_MAX_SPEED_RPM, 0.2, 30);

        wheelMotorEncoder = EncoderSim.createForChannel(wheelEncIdx);
        angleMotorEncoder = EncoderSim.createForChannel(azmthEncIdx);
    }

    public void update(boolean isDisabled){
        double wheelCmd = 0;
        double azmthCmd = 0;

        if(!isDisabled){
            wheelCmd = wheelMotorCtrl.getSpeed();
            azmthCmd = azmthMotorCtrl.getSpeed();
        }

        motionModel(wheelCmd, azmthCmd, 12.5 ); //hardcode full battery voltage for now.

        wheelMotorEncoder.setCount((int)Math.round(wheelMotor.getPosition_Rev() * Constants.WHEEL_ENC_COUNTS_PER_WHEEL_REV));
        angleMotorEncoder.setCount((int)Math.round(azmthMotor.getPosition_Rev() * Constants.AZMTH_ENC_COUNTS_PER_MODULE_REV));
    }

    public void motionModel(double wheelCmd, double angleCmd, double batteryVoltage_v){
        wheelMotor.update(batteryVoltage_v, wheelCmd, 0.0);
        azmthMotor.update(batteryVoltage_v, angleCmd, 0.0);

        double curAzmthAngleDeg = azmthMotor.getPosition_Rev() * 360;

        setMeasurements(wheelMotor.getSpeed_RPM(), curAzmthAngleDeg);
    }

    public void setMeasurements(double wheelSpeed_RPM, double angle_deg){
        curLinearSpeed_mps = UnitUtils.DtRPMtoMPerSec(wheelSpeed_RPM);
        curAzmthAngle = Rotation2d.fromDegrees(angle_deg);
    }

    public double getCurrentDraw_A(){
        return wheelMotor.getCurrent_A() + azmthMotor.getCurrent_A();
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(curLinearSpeed_mps, curAzmthAngle);
    } 

}