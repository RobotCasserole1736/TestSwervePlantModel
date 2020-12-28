
package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import frc.Constants;

class DrivetrainControl {

    SwerveModuleControl moduleFL;
    SwerveModuleControl moduleFR;
    SwerveModuleControl moduleBL;
    SwerveModuleControl moduleBR;


    double curActualSpeed_ftpersec = 0;
    double fwdRevSpdCmd = 0;
    double strafeSpdCmd = 0;
    double rotateSpdCmd = 0;

    public DrivetrainControl(){

        moduleFL = new SwerveModuleControl("FL", 0,1,0,2);
        moduleFR = new SwerveModuleControl("FR", 2,3,4,6);
        moduleBL = new SwerveModuleControl("BL", 4,5,8,10);
        moduleBR = new SwerveModuleControl("BR", 6,7,12,14);          

    }

    //TODO - add inputs for commanded fwd/rev, strafe, and rotate command

    //TODO - add input for setting desired pose

    //TODO somewhere else - add pathplanner stuff for swerve to calcualte a series of desired poses

    public void setInputs(double fwdRevCmd, double strafeCmd, double rotateCmd, double curActualSpeed){
        fwdRevSpdCmd = fwdRevCmd;
        strafeSpdCmd = strafeCmd;
        rotateSpdCmd = rotateCmd;
        curActualSpeed_ftpersec = curActualSpeed;
    }

    public void update(){

        var fwdRevSpd = Math.round(Timer.getFPGATimestamp()) % 10 < 5 ? 2.0 : -2.0;
        var translateSpd = Math.round(Timer.getFPGATimestamp()) % 4 < 2 ? 1.0 : -1.0;


        ChassisSpeeds desChSpd = new ChassisSpeeds(fwdRevSpdCmd, strafeSpdCmd, rotateSpdCmd);

        SwerveModuleState[] desModState = Constants.m_kinematics.toSwerveModuleStates(desChSpd);
        moduleFL.setDesiredState(desModState[0]);
        moduleFR.setDesiredState(desModState[1]);
        moduleBL.setDesiredState(desModState[2]);
        moduleBR.setDesiredState(desModState[3]);

        moduleFL.update(curActualSpeed_ftpersec);
        moduleFR.update(curActualSpeed_ftpersec);
        moduleBL.update(curActualSpeed_ftpersec);
        moduleBR.update(curActualSpeed_ftpersec);
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