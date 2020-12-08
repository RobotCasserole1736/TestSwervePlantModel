package frc.sim;

import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import frc.Constants;
import frc.UnitUtils;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;

class SwerveModuleModel{

    PWMSim wheelMotorCtrl;
    PWMSim azmthMotorCtrl;

    SimQuadratureEncoder wheelMotorEncoder;
    SimQuadratureEncoder angleMotorEncoder;

    double curLinearSpeed_mps = 0; //Positive = in curAngle_deg, Negative = opposite of curAngle_deg
    Rotation2d curAzmthAngle = Rotation2d.fromDegrees(0); //0 = toward front, 90 = toward left, 180 = toward back, 270 = toward right

    SimpleMotorWithMassModel wheelMotor;
    SimpleMotorWithMassModel azmthMotor;

    final double WHEEL_GEAR_RATIO = 6.1;
    final double AZMTH_GEAR_RATIO = 15.0;

    final double WHEEL_MOI = 0.1;
    final double AZMTH_MOI = 0.1;


    public SwerveModuleModel(int wheelMotorIdx, int azmthMotorIdx, int wheelEncIdx, int azmthEncIdx){
        wheelMotorCtrl = new PWMSim(wheelMotorIdx);
        azmthMotorCtrl = new PWMSim(azmthMotorIdx);

        wheelMotor = new SimpleMotorWithMassModel(DCMotor.getNEO(1), WHEEL_GEAR_RATIO, WHEEL_MOI);
        azmthMotor = new SimpleMotorWithMassModel(DCMotor.getVex775Pro(1), AZMTH_GEAR_RATIO, AZMTH_MOI);

        wheelMotorEncoder = new SimQuadratureEncoder(wheelEncIdx, wheelEncIdx + 1, Constants.ENC_PULSE_PER_REV, Constants.AZMTH_ENC_MODULE_REVS_PER_COUNT);
        angleMotorEncoder = new SimQuadratureEncoder(azmthEncIdx, azmthEncIdx + 1, Constants.ENC_PULSE_PER_REV, Constants.WHEEL_ENC_WHEEL_REVS_PER_COUNT);
    }

    public void update(boolean isDisabled){
        double wheelCmd = 0;
        double azmthCmd = 0;

        if(!isDisabled){
            wheelCmd = wheelMotorCtrl.getSpeed();
            azmthCmd = azmthMotorCtrl.getSpeed();
        }

        motionModel(wheelCmd, azmthCmd, 12.5 ); //hardcode full battery voltage for now.

        wheelMotorEncoder.setShaftPositionRev(wheelMotor.getPosition_Rev());
        angleMotorEncoder.setShaftPositionRev(azmthMotor.getPosition_Rev());
    }

    public void motionModel(double wheelCmd, double angleCmd, double batteryVoltage_v){
        wheelMotor.update(batteryVoltage_v, wheelCmd);
        azmthMotor.update(batteryVoltage_v, angleCmd);

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