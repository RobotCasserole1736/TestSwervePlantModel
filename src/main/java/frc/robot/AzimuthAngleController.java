package frc.robot;

import edu.wpi.first.wpilibj.controller.PIDController;
import frc.UnitUtils;
import frc.lib.DataServer.Annotations.Signal;
import frc.lib.Util.MapLookup2D;


public class AzimuthAngleController{

    PIDController azmthPIDCtrl = new PIDController(0.01, 0, 0.0001);

    double desAng = 0;

    @Signal(units = "deg")
    double actAng = 0;
    @Signal(units = "deg")
    double angSetpoint = 0;

    @Signal(units = "deg")
    double errNoInvert;

    @Signal(units = "deg")
    double errInvert;

    @Signal(units = "count")
    double actAngleHalfRotations;

    double netSpeed = 0;

    @Signal(units = "cmd")
    double azmthMotorCmd;

    @Signal
    boolean invertWheelDirection = false;

    MapLookup2D azmthCmdLimitTbl;


    public AzimuthAngleController(){

        azmthCmdLimitTbl = new MapLookup2D();
        azmthCmdLimitTbl.insertNewPoint(0.0, 1.0);
        azmthCmdLimitTbl.insertNewPoint(1.0, 1.0);
        azmthCmdLimitTbl.insertNewPoint(3.0, 0.5);
        azmthCmdLimitTbl.insertNewPoint(5.0, 0.1);
        azmthCmdLimitTbl.insertNewPoint(9.0, 0.1);

    }

    public void setInputs(double desiredAngle_in, double actualAngle_in, double curSpeed_fps_in){
        desAng = desiredAngle_in;
        actAng = actualAngle_in;
        netSpeed = curSpeed_fps_in; 
    }

    public void update(){

        errNoInvert = UnitUtils.wrapAngleDeg(desAng - actAng);
        errInvert   = UnitUtils.wrapAngleDeg(desAng - actAng + 180);

        if(Math.abs(errNoInvert) < Math.abs(errInvert)){
            //don't invert the wheel direction
            angSetpoint = actAng + errNoInvert;
            invertWheelDirection = false;
        } else {
            //Invert wheel direction
            angSetpoint = actAng + errInvert;
            invertWheelDirection = true;
        }

        azmthMotorCmd = azmthPIDCtrl.calculate(actAng, angSetpoint);
    }

    public double getMotorCmd(){
        return azmthMotorCmd;
    }

    public boolean getInvertWheelCmd(){
        return invertWheelDirection;
    }

    public double getErrMag_deg(){
        return Math.abs(angSetpoint - actAng);
    }

    public double getSetpoint_deg(){
        return angSetpoint;
    }


}