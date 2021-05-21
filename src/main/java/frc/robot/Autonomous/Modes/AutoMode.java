package frc.robot.Autonomous.Modes;

import frc.lib.AutoSequencer.AutoSequencer;

public abstract class AutoMode {

    public String humanReadableName = "";

    public abstract void addStepsToSequencer(AutoSequencer seq);
    
}
