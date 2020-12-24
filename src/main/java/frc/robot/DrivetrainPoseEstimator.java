
package frc.robot;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.util.Units;
import frc.Constants;
import frc.lib.DataServer.Annotations.Signal;
import frc.sim.simPhotonCam.PhotonPipelineResult;

class DrivetrainPoseEstimator {

    DrivetrainControl dt;

    Pose2d curEstPose = new Pose2d();

    PhotonCamera cam;

    WrapperedADXRS450 gyro;

    SwerveDriveOdometry m_odometry;

    @Signal(units = "ft/sec")
    double curSpeed = 0;

    public DrivetrainPoseEstimator(DrivetrainControl dt_in){
        dt = dt_in;
        gyro = new WrapperedADXRS450();
        cam = new PhotonCamera(Constants.PHOTON_CAM_NAME);
        m_odometry = new SwerveDriveOdometry(Constants.m_kinematics, getGyroHeading(), Constants.START_POSE);
        setKnownPose(Constants.START_POSE);
    }

    /**
     * Snap update the estimator to a known pose.
     * @param in known pose
     */
    public void setKnownPose(Pose2d in){
        gyro.reset();
        m_odometry.resetPosition(in, getGyroHeading());
    }

    public Pose2d getEstPose(){ return curEstPose; }

    public void update(){

        //Based on gyro and measured module speeds and positions, estimate where our robot should have moved to.
        SwerveModuleState[] states = dt.getModuleActualStates();
        Pose2d prevEstPose = curEstPose;
        curEstPose = m_odometry.update(getGyroHeading(), states[0], states[1], states[2], states[3]);

        //If we see a vision target, adjust our pose estimate
        var res = cam.getLatestResult();
        if(res.hasTargets()){
            System.out.println()
        }
        
        //Calculate a "speedometer" velocity in ft/sec
        Transform2d deltaPose = new Transform2d(prevEstPose, curEstPose);
        curSpeed = Units.metersToFeet(deltaPose.getTranslation().getNorm()) / Constants.CTRLS_SAMPLE_RATE_SEC;
    }

    public Rotation2d getGyroHeading(){
        return Rotation2d.fromDegrees(-1.0*gyro.getAngle());
    }

    public double getSpeedFtpSec(){
        return curSpeed;
    }


}