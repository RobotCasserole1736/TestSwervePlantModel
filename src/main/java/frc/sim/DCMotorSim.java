package frc.sim;

import edu.wpi.first.wpilibj.system.plant.DCMotor;

public class DCMotorSim {

    DCMotor motorParams;
    double curCurrent_A = 0;
    double prevCurrent_A = 0;
    double curTorque_Nm = 0;

    public DCMotorSim(DCMotor motorParams_in){
        motorParams = motorParams_in;
    }

    public void update(double speed_radpsec, double voltage_v){
        //TODO - motor is currently massless
        prevCurrent_A = curCurrent_A;
        curCurrent_A = motorParams.getCurrent(speed_radpsec, voltage_v);
        curTorque_Nm = motorParams.m_KtNMPerAmp * curCurrent_A;
    }

    public double getTorque_Nm(){
        return curTorque_Nm;
    }

    public double getCurrent_A(){
        return curCurrent_A;
    }
}