
package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import frc.Constants;
import frc.UnitUtils;
import frc.lib.DataServer.Signal;

class SwerveModuleControl {

    Spark wheelMotorCtrl;
    Spark azmthMotorCtrl;
    Encoder wheelEnc;
    Encoder azmthEnc;

    SwerveModuleState desState = new SwerveModuleState();
    SwerveModuleState actState = new SwerveModuleState();

    Signal wheelSpdDesSig;
    Signal wheelSpdActSig;
    Signal azmthPosDesSig;
    Signal azmthPosActSig;

    double wheelMotorSpeedDes_RPM = 0;
    double wheelMotorSpeedAct_RPM = 0;

    double azmthPosDes_deg = 0;
    double azmthPosAct_deg = 0;

    public SwerveModuleControl(String posId, int wheelMotorIdx, int azmthMotorIdx, int wheelEncoderIdx, int azmthEncoderIdx){

        wheelMotorCtrl = new Spark(wheelMotorIdx);
        azmthMotorCtrl = new Spark(azmthMotorIdx);
        wheelEnc = new Encoder(wheelEncoderIdx, wheelEncoderIdx + 1); //Always assume channel B is one after channel A.
        azmthEnc = new Encoder(azmthEncoderIdx, azmthEncoderIdx + 1);

        wheelEnc.setDistancePerPulse(Constants.WHEEL_ENC_WHEEL_REVS_PER_COUNT);
        azmthEnc.setDistancePerPulse(Constants.AZMTH_ENC_MODULE_REVS_PER_COUNT);

        wheelSpdDesSig = new Signal("DtModule" + posId + "WheelSpdDes", "RPM");
        wheelSpdActSig = new Signal("DtModule" + posId + "WheelSpdAct", "RPM");
        azmthPosDesSig = new Signal("DtModule" + posId + "AzmthPosDes", "deg");
        azmthPosActSig = new Signal("DtModule" + posId + "AzmthPosAct", "deg");

    }

    public void update(){

        wheelMotorSpeedDes_RPM = UnitUtils.DtMPerSectoRPM(desState.speedMetersPerSecond);
        wheelMotorSpeedAct_RPM = wheelEnc.getRate() * 60;

        azmthPosDes_deg = desState.angle.getDegrees();
        azmthPosAct_deg = azmthEnc.getDistance() * 360.0;

        //TODO - PID or Open-Loop control of wheel velocity
        
        //TODO - apply azimuth velocity rate limit based on measured wheel velocity

        //TODO - magic-motion or similar control of azimuth angle

        //TODO - maybe - switch-mode PID for position control when within ~2 degrees of target? Maybe? If magic-motion won't lock it in place?

        wheelMotorCtrl.set(0.75); //Simple silly open-loop circle-ish control law
        azmthMotorCtrl.set(0.25); //Simple silly open-loop circle-ish control law

        updateTelemetry();
    }

    public void updateTelemetry(){
        double sampleTimeMs = LoopTiming.getInstance().getLoopStartTimeSec() * 1000;
        wheelSpdDesSig.addSample(sampleTimeMs, wheelMotorSpeedDes_RPM);
        wheelSpdActSig.addSample(sampleTimeMs, wheelMotorSpeedAct_RPM);
        azmthPosDesSig.addSample(sampleTimeMs, azmthPosDes_deg);
        azmthPosActSig.addSample(sampleTimeMs, azmthPosAct_deg);
    }

    //TODO - test mode update for PID tuning azimuth motor velocity

    public void setDesiredState(SwerveModuleState des){
        desState = des;
    }

    public SwerveModuleState getActualState(){
        return actState;
    }

    public SwerveModuleState getDesiredState(){
        return desState;
    }

}
