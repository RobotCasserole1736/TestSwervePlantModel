package frc.robot.Autonomous.Modes.Events;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Units;
import frc.UnitUtils;
import frc.lib.AutoSequencer.AutoEvent;
import frc.robot.Drivetrain.DrivetrainControl;

public class AutoEventRotateTime extends AutoEvent {
	double duration_s;
	double endTime;

	double speed_dps;
	boolean done = false;

	public AutoEventRotateTime(double duration_s_in, double speed_dps_in) {
        duration_s = duration_s_in;
        speed_dps = speed_dps_in;
	}

	@Override
	public void userStart() {
        endTime = Timer.getFPGATimestamp() + duration_s;
        done = false;
	}

	@Override
	public void userUpdate() {
        if(Timer.getFPGATimestamp() < endTime){
            DrivetrainControl.getInstance().setInputs(0, 0, Units.degreesToRadians(speed_dps));
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
}
