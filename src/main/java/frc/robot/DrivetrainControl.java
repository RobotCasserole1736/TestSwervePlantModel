
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

        moduleFL = new SwerveModuleControl("FL", Constants.FL_WHEEL_MOTOR_IDX,Constants.FL_AZMTH_MOTOR_IDX,Constants.FL_WHEEL_ENC_A_IDX,Constants.FL_AZMTH_ENC_A_IDX);
        moduleFR = new SwerveModuleControl("FR", Constants.FR_WHEEL_MOTOR_IDX,Constants.FR_AZMTH_MOTOR_IDX,Constants.FR_WHEEL_ENC_A_IDX,Constants.FR_AZMTH_ENC_A_IDX);
        moduleBL = new SwerveModuleControl("BL", Constants.BL_WHEEL_MOTOR_IDX,Constants.BL_AZMTH_MOTOR_IDX,Constants.BL_WHEEL_ENC_A_IDX,Constants.BL_AZMTH_ENC_A_IDX);
        moduleBR = new SwerveModuleControl("BR", Constants.BR_WHEEL_MOTOR_IDX,Constants.BR_AZMTH_MOTOR_IDX,Constants.BR_WHEEL_ENC_A_IDX,Constants.BR_AZMTH_ENC_A_IDX);          

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

        double worstError = getMaxErrorMag();

        moduleFL.update(curActualSpeed_ftpersec, worstError);
        moduleFR.update(curActualSpeed_ftpersec, worstError);
        moduleBL.update(curActualSpeed_ftpersec, worstError);
        moduleBR.update(curActualSpeed_ftpersec, worstError);
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

    public double getMaxErrorMag(){
        double maxErr = 0;
        maxErr = Math.max(maxErr, moduleFL.azmthCtrl.getErrMag_deg());
        maxErr = Math.max(maxErr, moduleFR.azmthCtrl.getErrMag_deg());
        maxErr = Math.max(maxErr, moduleBL.azmthCtrl.getErrMag_deg());
        maxErr = Math.max(maxErr, moduleFR.azmthCtrl.getErrMag_deg());
        return maxErr;
    }

}