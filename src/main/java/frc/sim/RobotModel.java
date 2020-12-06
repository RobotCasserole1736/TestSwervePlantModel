package frc.sim;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;

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
    }

    public Pose2d getCurActPose(){
        return dt.field.getRobotObject().getPose();
    }

}