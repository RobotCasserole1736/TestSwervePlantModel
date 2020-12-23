
package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import frc.Constants;
import frc.UnitUtils;
import frc.lib.DataServer.Annotations.Signal;

class SwerveModuleControl {

    Spark wheelMotorCtrl;
    Spark azmthMotorCtrl;
    Encoder wheelEnc;
    Encoder azmthEnc;

    SwerveModuleState desState = new SwerveModuleState();
    SwerveModuleState actState = new SwerveModuleState();

    frc.lib.DataServer.Signal wheelSpdDesSig;
    frc.lib.DataServer.Signal wheelSpdActSig;
    frc.lib.DataServer.Signal azmthPosDesSig;
    frc.lib.DataServer.Signal azmthPosActSig;

    double wheelMotorSpeedDes_RPM = 0;
    double wheelMotorSpeedAct_RPM = 0;

    double azmthPosDes_deg = 0;
    double azmthPosAct_deg = 0;

    @Signal
    boolean invertWheelDirection = false;

    @Signal(units = "cmd")
    double wheelMotorCmd;
    @Signal(units = "cmd")
    double azmthMotorCmd;

    final double WHEEL_MAX_SPEED_RPM = 620; //determined empirically

    PIDController wheelPIDCtrl = new PIDController(0.011, 0, 0.001);
    PIDController azmthPIDCtrl = new PIDController(0.01, 0, 0.0001);

    public SwerveModuleControl(String posId, int wheelMotorIdx, int azmthMotorIdx, int wheelEncoderIdx, int azmthEncoderIdx){

        wheelMotorCtrl = new Spark(wheelMotorIdx);
        azmthMotorCtrl = new Spark(azmthMotorIdx);
        wheelEnc = new Encoder(wheelEncoderIdx, wheelEncoderIdx + 1); //Always assume channel B is one after channel A.
        azmthEnc = new Encoder(azmthEncoderIdx, azmthEncoderIdx + 1);

        wheelEnc.setDistancePerPulse(Constants.WHEEL_ENC_WHEEL_REVS_PER_COUNT);
        azmthEnc.setDistancePerPulse(Constants.AZMTH_ENC_MODULE_REVS_PER_COUNT);

        azmthPIDCtrl.enableContinuousInput(-180.0, 180.0);

        wheelSpdDesSig = new frc.lib.DataServer.Signal("DtModule" + posId + "WheelSpdDes", "RPM");
        wheelSpdActSig = new frc.lib.DataServer.Signal("DtModule" + posId + "WheelSpdAct", "RPM");
        azmthPosDesSig = new frc.lib.DataServer.Signal("DtModule" + posId + "AzmthPosDes", "deg");
        azmthPosActSig = new frc.lib.DataServer.Signal("DtModule" + posId + "AzmthPosAct", "deg");

    }

    public void update(){

        double azmthPosDesMotorNorm_deg = UnitUtils.wrapAngleDeg(desState.angle.getDegrees());
        double azmthPosDesMotorInv_deg = UnitUtils.wrapAngleDeg(azmthPosDesMotorNorm_deg + 180);
        azmthPosAct_deg = UnitUtils.wrapAngleDeg(azmthEnc.getDistance() * 360.0);

        double normalErr = UnitUtils.wrapAngleDeg(azmthPosDes_deg - azmthPosAct_deg);
        double invMotorErr = UnitUtils.wrapAngleDeg(azmthPosDesMotorInv_deg - azmthPosAct_deg);

        if(Math.abs(normalErr) < Math.abs(invMotorErr)){
            azmthPosDes_deg = azmthPosDesMotorNorm_deg;
            invertWheelDirection = false;
        } else {
            azmthPosDes_deg = azmthPosDesMotorInv_deg;
            invertWheelDirection = true;
        }

        wheelMotorSpeedDes_RPM = UnitUtils.DtMPerSectoRPM(desState.speedMetersPerSecond)*(invertWheelDirection?-1.0:1.0);
        wheelMotorSpeedAct_RPM = wheelEnc.getRate() * 60;

        
        //TODO - apply azimuth velocity rate limit based on measured wheel velocity

        //TODO - magic-motion or similar control of azimuth angle

        //TODO - maybe - switch-mode PID for position control when within ~2 degrees of target? Maybe? If magic-motion won't lock it in place?

        //Closed-loop control of Azimuth position
        azmthPIDCtrl.setSetpoint(azmthPosDes_deg);
        azmthMotorCmd = UnitUtils.limitMotorCmd(azmthPIDCtrl.calculate(azmthPosAct_deg));

        //Closed-loop control of wheel velocity
        wheelPIDCtrl.setSetpoint(wheelMotorSpeedDes_RPM);
        double wheelFFCmd = wheelMotorSpeedDes_RPM/WHEEL_MAX_SPEED_RPM;
        double wheelFBCmd = wheelPIDCtrl.calculate(wheelMotorSpeedAct_RPM);
        wheelMotorCmd = UnitUtils.limitMotorCmd(wheelFFCmd+wheelFBCmd);

        //wheelMotorCtrl.set(wheelMotorCmd); 
        wheelMotorCtrl.set(0); 
        //azmthMotorCtrl.set(azmthMotorCmd); 
        azmthMotorCtrl.set(0); 

        updateTelemetry();
    }

    /**
     * Broadcast signals specific to the visualiation
     */
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
