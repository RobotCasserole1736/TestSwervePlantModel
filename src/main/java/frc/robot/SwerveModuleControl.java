
package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import frc.Constants;

class SwerveModuleControl {

    Spark wheelMotorCtrl;
    Spark azmthMotorCtrl;
    Encoder wheelEnc;
    Encoder azmthEnc;

    SwerveModuleState desState;
    SwerveModuleState actState;

    public SwerveModuleControl(String posId, int wheelMotorIdx, int azmthMotorIdx, int wheelEncoderIdx, int azmthEncoderIdx){

        wheelMotorCtrl = new Spark(wheelMotorIdx);
        azmthMotorCtrl = new Spark(azmthMotorIdx);
        wheelEnc = new Encoder(wheelEncoderIdx, wheelEncoderIdx + 1); //Always assume channel B is one after channel A.
        azmthEnc = new Encoder(azmthEncoderIdx, azmthEncoderIdx + 1);

        wheelEnc.setDistancePerPulse(Constants.WHEEL_ENC_WHEEL_REVS_PER_COUNT);
        azmthEnc.setDistancePerPulse(Constants.AZMTH_ENC_MODULE_REVS_PER_COUNT);

    }

    public void update(){
        wheelMotorCtrl.set(0.25); //Simple silly open-loop circle-ish control law
        azmthMotorCtrl.set(0.25); //Simple silly open-loop circle-ish control law

    }

    public void setDesiredState(SwerveModuleState des){
        desState = des;
    }

    public SwerveModuleState getActualState(){
        return actState;
    }

}
