
package frc.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;

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