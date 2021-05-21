package frc.robot.Autonomous.Modes.Events;

import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.Constants;
import frc.lib.AutoSequencer.AutoEvent;
import frc.robot.Drivetrain.DrivetrainControl;

public class AutoEventDriveFig8 extends AutoEvent {

	double startTime = 0;
	boolean done = false;

	TrajectoryConfig config;
	Trajectory traj;
	
	public AutoEventDriveFig8() {
		config = new TrajectoryConfig(Constants.MAX_FWD_REV_SPEED_MPS, Constants.MAX_TRANSLATE_ACCEL_MPS2);
		config.setKinematics(Constants.m_kinematics);
		traj = TrajectoryGenerator.generateTrajectory( List.of(
			new Pose2d(new Translation2d(3.0, 3.0), Rotation2d.fromDegrees(0)),
			new Pose2d(new Translation2d(5.0, 1.0 ), Rotation2d.fromDegrees(0)),
			new Pose2d(new Translation2d(7.0, 5.0), Rotation2d.fromDegrees(0)),
			new Pose2d(new Translation2d(9.0, 3.0), Rotation2d.fromDegrees(-90)),
			new Pose2d(new Translation2d(7.0, 1.0 ), Rotation2d.fromDegrees(-180)),
			new Pose2d(new Translation2d(5.0, 5.0), Rotation2d.fromDegrees(-180)),
			new Pose2d(new Translation2d(3.0, 3.0), Rotation2d.fromDegrees(-180))
		), config);
	}

	@Override
	public void userStart() {
		startTime = Timer.getFPGATimestamp();
        done = false;
	}

	@Override
	public void userUpdate() {
		var deltaTime = Timer.getFPGATimestamp() - startTime;

		if(deltaTime < traj.getTotalTimeSeconds()){
			Trajectory.State dtCmd = traj.sample(deltaTime);
			DrivetrainControl.getInstance().setInputs(dtCmd);
			done = false;
		} else {
			DrivetrainControl.getInstance().stop();
			done = true;
		}
		return;
	}

	@Override
	public void userForceStop() {
        DrivetrainControl.getInstance().stop();
        done = true;
		return;
	}

	@Override
	public boolean isTriggered() {
		return true;
	}

	@Override
	public boolean isDone() {
		return done;
	}

	public Pose2d getStartPose(){
		return traj.getInitialPose();
	}

}
