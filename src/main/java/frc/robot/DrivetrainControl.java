
package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import frc.Constants;

class DrivetrainControl {

    SwerveModuleControl moduleFL;
    SwerveModuleControl moduleFR;
    SwerveModuleControl moduleBL;
    SwerveModuleControl moduleBR;

    public DrivetrainControl(){

        moduleFL = new SwerveModuleControl("FL", 0,1,0,2);
        moduleFR = new SwerveModuleControl("FR", 2,3,4,6);
        moduleBL = new SwerveModuleControl("BL", 4,5,8,10);
        moduleBR = new SwerveModuleControl("BR", 6,7,12,14);          

    }

    //TODO - add inputs for commanded fwd/rev, strafe, and rotate command

    //TODO - add input for setting desired pose

    //TODO somewhere else - add pathplanner stuff for swerve to calcualte a series of desired poses

    //TODO somewhere else - add class to generate telemetry signals for all swerve module states.

    public void update(){

        var fwdRevSpd = Math.round(Timer.getFPGATimestamp()) % 2 < 1 ? 1.0 : 1.0;
        var translateSpd = Math.round(Timer.getFPGATimestamp()) % 3 < 1 ? 0.5 : -0.5;
        ChassisSpeeds desChSpd = new ChassisSpeeds(fwdRevSpd, translateSpd, 1);

        SwerveModuleState[] desModState = Constants.m_kinematics.toSwerveModuleStates(desChSpd);
        moduleFL.setDesiredState(desModState[0]);
        moduleFR.setDesiredState(desModState[1]);
        moduleBL.setDesiredState(desModState[2]);
        moduleBR.setDesiredState(desModState[3]);

        //TODO  - add logic to convert desired fwd/rev/strafe/rotate commadns to module states- 

        //TODO - add closed-loop logic to convert error between estimated and desired state into module desired states.

        moduleFL.update();
        moduleFR.update();
        moduleBL.update();
        moduleBR.update();

        //TODO - add logic to read module state and update an estimated position
    }


    public SwerveModuleState [] getModuleActualStates(){
        SwerveModuleState retArr[] =  { moduleFL.getActualState(),
                                        moduleFR.getActualState(),
                                        moduleBL.getActualState(),
                                        moduleFR.getActualState()};
        return retArr;
    }

    public SwerveModuleState [] getModuleDesiredStates(){
        SwerveModuleState retArr[] =  { moduleFL.getDesiredState(),
                                        moduleFR.getDesiredState(),
                                        moduleBL.getDesiredState(),
                                        moduleFR.getDesiredState()};
        return retArr;
    }

}