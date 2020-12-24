package frc.sim;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import frc.Constants;
import frc.sim.simPhotonCam.SimPhotonCamera;

public class SimVisionModel{

    SimPhotonCamera photonCam;

    final Pose2d CLOSE_ALLIANCE_TARGET_POSE = new Pose2d();
    final Pose2d FAR_ALLIANCE_TARGET_POSE = new Pose2d();

    public SimVisionModel(){
        photonCam = new SimPhotonCamera(Constants.PHOTON_CAM_NAME);
    }

    public void update(Pose2d curRobotPose){

        photonCam.clearAllTargets();

        //Todo - update camera state based on robot pose

        //If the pose is close enough to one of the targets and pointed within the camera's FOV, report the target

        //Otherwise, report no targets visible

        //Pass our model of what the photonCam would be reading back into the 
        // sim photon camera.

    }
}