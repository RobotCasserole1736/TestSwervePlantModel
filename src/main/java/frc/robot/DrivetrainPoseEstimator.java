
package frc.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.geometry.Pose2d;

class DrivetrainPoseEstimator {

    DrivetrainControl dt;

    Pose2d curEstPose = new Pose2d();

    //TODO - add photonvision camera

    ADXRS450_Gyro gyro;



    public DrivetrainPoseEstimator(DrivetrainControl dt_in){
        dt = dt_in;
        gyro = new ADXRS450_Gyro();

    }

    /**
     * Snap update the estimator to a known pose.
     * @param in known pose
     */
    public void setKnownPose(Pose2d in){
        curEstPose = in; 
    }

    public Pose2d getEstPose(){ return curEstPose; }

    public void update(){

        //TODO - add logic to read module state and update an estimated position
    }


}