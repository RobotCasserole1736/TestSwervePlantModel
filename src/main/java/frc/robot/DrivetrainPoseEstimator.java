
package frc.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import frc.Constants;

class DrivetrainPoseEstimator {

    DrivetrainControl dt;

    Pose2d curEstPose = new Pose2d();

    //TODO - add photonvision camera

    WrapperedADXRS450 gyro;

    SwerveDriveOdometry m_odometry;

    public DrivetrainPoseEstimator(DrivetrainControl dt_in){
        dt = dt_in;
        gyro = new WrapperedADXRS450();
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

        SwerveModuleState[] states = dt.getModuleActualStates();
        curEstPose = m_odometry.update(getGyroHeading(), states[0], states[1], states[2], states[3]);
    }

    public Rotation2d getGyroHeading(){
        return Rotation2d.fromDegrees(-1.0*gyro.getAngle());
    }


}