
package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPipelineResult;
import org.photonvision.PhotonUtils;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.util.Units;
import frc.lib.DataServer.Annotations.Signal;

class DrivetrainPoseEstimator {

    DrivetrainControl dt;

    Pose2d curEstPose = new Pose2d();

    PhotonCamera camera;

    ADXRS450_Gyro gyro;

    // Constants
    static final double kCameraHeight = Units.inchesToMeters(36.5 + 6.25);
    static final double kCameraPitch = Units.degreesToRadians(14);
    static final double kTargetHeight = Units.inchesToMeters(57);

    // Get distance to target.
    @Signal(units = "in")
    double distanceToTarget;

    @Signal(units="ms")
    double camLatencySec;

    //Local photonvision connection
    NetworkTableInstance inst;
    NetworkTable table;

    public DrivetrainPoseEstimator(DrivetrainControl dt_in) {
        dt = dt_in;
        gyro = new ADXRS450_Gyro();

        if(Robot.isReal()){
            camera = new PhotonCamera("mmal_service_16.1");
        } else {
            //Local test without real RIO - connect to photon camera which is hosting the NT instance
            inst = NetworkTableInstance.create();
            table = inst.getTable("photonvision").getSubTable("mmal_service_16.1");
            inst.startClient("10.111.76.70");
            camera = new PhotonCamera(table);
        }

    }

    /**
     * Snap update the estimator to a known pose.
     * 
     * @param in known pose
     */
    public void setKnownPose(Pose2d in) {
        curEstPose = in;
    }

    public Pose2d getEstPose() {
        return curEstPose;
    }

    public void update(){
        PhotonPipelineResult res = camera.getLatestResult();

        camLatencySec = res.getLatencyMillis();

        if(res.hasTargets()){
            distanceToTarget = Units.metersToInches(PhotonUtils.calculateDistanceToTargetMeters(
                kCameraHeight, kTargetHeight, kCameraPitch, Units.degreesToRadians(res.getBestTarget().getPitch())));
        } else {
            distanceToTarget = -1.0;
        }

    }


}