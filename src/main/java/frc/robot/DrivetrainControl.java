
package frc.robot;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
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

        SwerveModuleState[] desModState;

        if(Math.abs(fwdRevSpdCmd) > 0.1 | Math.abs(strafeSpdCmd) > 0.1 | Math.abs(rotateSpdCmd) > 0.1){
            //In motion
            ChassisSpeeds desChSpd = new ChassisSpeeds(fwdRevSpdCmd, strafeSpdCmd, rotateSpdCmd);
            desModState = Constants.m_kinematics.toSwerveModuleStates(desChSpd);
        } else {
            //Home Position
            desModState = new SwerveModuleState[4];
            desModState[0] = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
            desModState[1] = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
            desModState[2] = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
            desModState[3] = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
        }

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