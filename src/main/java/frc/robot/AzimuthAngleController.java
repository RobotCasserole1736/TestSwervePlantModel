package frc.robot;

import edu.wpi.first.wpilibj.controller.PIDController;
import frc.lib.DataServer.Annotations.Signal;
import frc.lib.Util.MapLookup2D;


public class AzimuthAngleController{

    PIDController azmthPIDCtrl = new PIDController(0.01, 0, 0.0001);

    @Signal(units = "cmd")
    double azmthMotorCmd;

    @Signal
    boolean invertWheelDirection = false;

    MapLookup2D azmthCmdLimitTbl;


    public AzimuthAngleController(){

        
        azmthPIDCtrl.enableContinuousInput(-180.0, 180.0);

        azmthCmdLimitTbl = new MapLookup2D();
        azmthCmdLimitTbl.insertNewPoint(0.0, 1.0);
        azmthCmdLimitTbl.insertNewPoint(1.0, 1.0);
        azmthCmdLimitTbl.insertNewPoint(3.0, 0.5);
        azmthCmdLimitTbl.insertNewPoint(5.0, 0.1);
        azmthCmdLimitTbl.insertNewPoint(9.0, 0.1);

    }

    public void setInputs(double desiredAngle, double actualAngle, double curSpeedFtPerSec){
        
    }

    public void update(){

    }

    public double getMotorCmd(){
        return azmthMotorCmd;
    }

    public boolean getInvertWheelCmd(){
        return invertWheelDirection;
    }


}