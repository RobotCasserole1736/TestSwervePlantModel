package frc.robot.Drivetrain;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.util.Units;
import frc.lib.DataServer.Signal;

public class DtPoseView {

    
    //Desired Position says where path planning logic wants the
    // robot to be at any given time. 
    Signal xPosDesFtSig;
    Signal yPosDesFtSig;
    Signal tRotDesDegSig;

    //Estimated position says where you think your robot is at
    // Based on encoders, motion, vision, etc.
    Signal xPosEstFtSig;
    Signal yPosEstFtSig;
    Signal tRotEstDegSig;

    //Actual position defines wherever the robot is actually at
    // at any time. It is unknowable in real life, but is a key
    // part of the simulation
    Signal xPosActFtSig;
    Signal yPosActFtSig;
    Signal tRotActDegSig;

    Pose2d actualPose = new Pose2d();
    Pose2d desiredPose = new Pose2d();
    Pose2d estimatedPose = new Pose2d();

    public DtPoseView(){
        xPosDesFtSig     = new Signal("botDesPoseX", "ft");
        yPosDesFtSig     = new Signal("botDesPoseY", "ft");
        tRotDesDegSig    = new Signal("botDesPoseT", "deg");

        xPosEstFtSig     = new Signal("botEstPoseX", "ft");
        yPosEstFtSig     = new Signal("botEstPoseY", "ft");
        tRotEstDegSig    = new Signal("botEstPoseT", "deg");

        xPosActFtSig     = new Signal("botActPoseX", "ft");
        yPosActFtSig     = new Signal("botActPoseY", "ft");
        tRotActDegSig    = new Signal("botActPoseT", "deg");
    }

    public void setActualPose(Pose2d act){
        actualPose = act;
    }
    public void setDesiredPose(Pose2d des){
        desiredPose = des;
    }
    public void setEstimatedPose(Pose2d est){
        estimatedPose = est;
    }

    public void update(double time){
        xPosActFtSig.addSample(time,  Units.metersToFeet(actualPose.getTranslation().getX()));
        yPosActFtSig.addSample(time,  Units.metersToFeet(actualPose.getTranslation().getY()));
        tRotActDegSig.addSample(time, actualPose.getRotation().getDegrees());

        xPosDesFtSig.addSample(time,  Units.metersToFeet(desiredPose.getTranslation().getX()));
        yPosDesFtSig.addSample(time,  Units.metersToFeet(desiredPose.getTranslation().getY()));
        tRotDesDegSig.addSample(time, desiredPose.getRotation().getDegrees());

        xPosEstFtSig.addSample(time,  Units.metersToFeet(estimatedPose.getTranslation().getX()));
        yPosEstFtSig.addSample(time,  Units.metersToFeet(estimatedPose.getTranslation().getY()));
        tRotEstDegSig.addSample(time, estimatedPose.getRotation().getDegrees());
    }




    
}