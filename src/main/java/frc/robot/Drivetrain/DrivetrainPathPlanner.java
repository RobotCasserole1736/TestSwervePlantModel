
package frc.robot.Drivetrain;

import java.io.File;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;

public class DrivetrainPathPlanner {


    /* Singleton infrastructure */
    private static DrivetrainPathPlanner instance;
    public static DrivetrainPathPlanner getInstance() {
        if (instance == null) {
            instance = new DrivetrainPathPlanner();
        }
        return instance;
    }


    double startTime = 0.0;

    private DrivetrainPathPlanner(){


    }

    /**
     * Establishes a new field-referenced path given a starting pose, ending pose, and desired time to complete.
     */
    public void initSimplePathFromPoses(Pose2d start, Pose2d end, double duration_sec){
        //TODO do something with all this
    }

    /**
     * Establishes a new field-referenced path given a pathweaver .json input
     */
    public void initPathFromJSON(File pathIn){
        //TODO do something with all this
    }


    /**
     * User should call this right as they start executing through a path
     * It marks the start time referenced from the FPGA timestamp
     */
    public void startPath(){
        startTime = Timer.getFPGATimestamp();
    }

    /**
     * Returns true if the path's duration has expired
     */
    public boolean isPathFinished(){
        return false;
    }

    public Pose2d getCurDesiredPose(){
        return new Pose2d(); //TODO - calcualte and return the desired pose right now
    }

    public void getCurDtCommand(){
        //TODO - return some drivetrain command.... maybe a translation/rotation pair?
    }

}