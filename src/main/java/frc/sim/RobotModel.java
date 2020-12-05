package frc.sim;

import edu.wpi.first.wpilibj.Timer;

public class RobotModel {

    private DrivetrainModel dt;

    public RobotModel(){
        dt = new DrivetrainModel();
        reset();
    }

    public void reset(){

        dt.modelReset();
    }

    public void update(boolean isDisabled){
        dt.update(isDisabled);

        double curTime_ms = Timer.getFPGATimestamp() * 1000;
        dt.updateTelemetry(curTime_ms);
    }

}