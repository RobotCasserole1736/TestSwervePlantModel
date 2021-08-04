package frc.robot.Autonomous.Modes;

import frc.lib.AutoSequencer.AutoSequencer;
import frc.robot.Autonomous.Modes.Events.AutoEventDriveFwdTime;
import frc.robot.Autonomous.Modes.Events.AutoEventDriveSidewaysTime;
import frc.robot.Autonomous.Modes.Events.AutoEventRotateTime;

public class AutoModeDriveForward3Sec extends AutoMode {

    public AutoModeDriveForward3Sec(){
        humanReadableName = "Drive Forward for 3 Sec";
    }

    @Override
    public void addStepsToSequencer(AutoSequencer seq){
        seq.addEvent(new AutoEventDriveFwdTime(1.0, 0.75));
        seq.addEvent(new AutoEventRotateTime(3.0, 90));
        seq.addEvent(new AutoEventDriveSidewaysTime(3.0, 0.75));
        seq.addEvent(new AutoEventDriveFwdTime(3.0, -0.5));

    }

    
}
