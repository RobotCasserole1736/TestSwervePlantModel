package frc.sim;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.util.Units;
import frc.Constants;
import frc.UnitUtils;
import frc.sim.simPhotonCam.SimPhotonCamera;

public class SimVisionModel{

    SimPhotonCamera photonCam;

    Pose2d fieldOrigin = new Pose2d();
    Pose2d closeTargetPose = fieldOrigin.transformBy(Constants.fieldToCloseVisionTargetTrans);
    Pose2d farTargetPose = fieldOrigin.transformBy(Constants.fieldToFarVisionTargetTrans);

    final double MAX_CAM_DISTANCE_M = Units.feetToMeters(20);
    final double CAM_HORIZ_FOV_DEG = 60.0;

    public SimVisionModel(){
        photonCam = new SimPhotonCamera(Constants.PHOTON_CAM_NAME);
    }

    public void update(Pose2d curRobotPose){

        photonCam.clearAllTargets();

        Pose2d camPose = curRobotPose.transformBy(Constants.robotToCameraTrans);

        var farTgtTrans = getCamToTgtTrans(farTargetPose, camPose);
        if(farTgtTrans != null){
            photonCam.reportDetectedTarget(farTgtTrans);
        }

        var closeTgtTrans = getCamToTgtTrans(closeTargetPose, camPose);
        if(closeTgtTrans != null){
            photonCam.reportDetectedTarget(closeTgtTrans);
        }

        photonCam.update();

    }

    public Transform2d getCamToTgtTrans(Pose2d targetPos, Pose2d cameraPos){
        var camToTargetTrans = new Transform2d(cameraPos, targetPos);
        boolean inRange = (camToTargetTrans.getTranslation().getNorm() < MAX_CAM_DISTANCE_M);
        boolean inAngle = Math.abs(UnitUtils.wrapAngleDeg(camToTargetTrans.getRotation().getDegrees())) < (CAM_HORIZ_FOV_DEG/2);
        if(inRange & inAngle){
            return camToTargetTrans;
        } else {
            return null;
        }
    }

}