package frc.robot.HumanInterface;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Units;
import frc.lib.DataServer.Annotations.Signal;

public class DriverInterface{


    public final double MAX_FWD_REV_SPEED_MPS = Units.feetToMeters(12.0);
    public final double MAX_STRAFE_SPEED_MPS = Units.feetToMeters(8.0);
    public final double MAX_ROTATE_SPEED_RAD_PER_SEC = Units.degreesToRadians(360.0);

    public final double MAX_TRANSLATE_ACCEL_MPS2 = MAX_FWD_REV_SPEED_MPS/0.25; //0-full time of 0.25 second
    public final double MAX_ROTATE_ACCEL_RAD_PER_SEC_2 = MAX_ROTATE_SPEED_RAD_PER_SEC/0.25; //0-full time of 0.25 second

    final double JOY_DEADBAND = 0.1;

    XboxController ctrl;

    @Signal(units="mps")
    double fwdRevSpdCmd = 0;
    @Signal(units="mps")
    double strafeSpdCmd = 0;
    @Signal(units="radPerSec")
    double rotateCmd = 0;

    SlewRateLimiter fwdRevSlew;
    SlewRateLimiter strafeSlew;
    SlewRateLimiter rotateSlew;

    public DriverInterface(){
        ctrl = new XboxController(0);

        fwdRevSlew = new SlewRateLimiter(MAX_TRANSLATE_ACCEL_MPS2);
        strafeSlew = new SlewRateLimiter(MAX_TRANSLATE_ACCEL_MPS2);
        rotateSlew = new SlewRateLimiter(MAX_ROTATE_ACCEL_RAD_PER_SEC_2);

    }

    public void update(){
        fwdRevSpdCmd = -1.0*applyDeadband(ctrl.getRawAxis(1))*MAX_FWD_REV_SPEED_MPS;
        strafeSpdCmd = -1.0*applyDeadband(ctrl.getRawAxis(0))*MAX_STRAFE_SPEED_MPS;
        rotateCmd    = -1.0*applyDeadband(ctrl.getRawAxis(2))*MAX_ROTATE_SPEED_RAD_PER_SEC;

        fwdRevSpdCmd = fwdRevSlew.calculate(fwdRevSpdCmd);
        strafeSpdCmd = strafeSlew.calculate(strafeSpdCmd);
        rotateCmd    = rotateSlew.calculate(rotateCmd);
    }

    public double getFwdRevSpeedCmd_mps(){
        return fwdRevSpdCmd;
    }

    public double getStrafeSpeedCmd_mps(){
        return strafeSpdCmd;
    }

    public double getRotateCmd_radPerSec(){
        return rotateCmd;
    }

    private double applyDeadband(double input){
        if(Math.abs(input) < JOY_DEADBAND){
            return 0.0;
        } else {
            return Math.signum(input) * ( (Math.abs(input) - JOY_DEADBAND) * 1.0 / (1.0 - JOY_DEADBAND) );
        }
        
    }
}
