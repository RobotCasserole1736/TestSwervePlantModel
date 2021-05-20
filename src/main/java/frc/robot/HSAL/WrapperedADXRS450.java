package frc.robot.HSAL;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import frc.lib.DataServer.Annotations.Signal;
import frc.robot.Robot;

/**
 * Our favorite ADXRS450 and 453 gyro doesn't yet fully support
 * all required features in sim, so we're adding this wrapper to
 * provide those features;
 */
public class WrapperedADXRS450 {

    ADXRS450_Gyro gyro;

    double readingOffset = 0;

    @Signal(units="deg")
    double rawGyroAngle = 0;

    public WrapperedADXRS450(){
        gyro = new ADXRS450_Gyro();
        
    } 

    public void reset(){
        if(Robot.isSimulation()){
            readingOffset = gyro.getAngle();
        } else {
            gyro.reset();
        }
    }

    public double getRate(){
        return gyro.getRate();
    }

    public double getAngle(){
        rawGyroAngle = gyro.getAngle() - readingOffset;
        return rawGyroAngle;
    }

    public void calibrate(){
        gyro.calibrate();
    }

    public boolean isConnected(){
        return gyro.isConnected();
    }


}