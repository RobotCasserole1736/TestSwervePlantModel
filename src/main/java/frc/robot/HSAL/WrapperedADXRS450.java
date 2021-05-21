package frc.robot.HSAL;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.lib.DataServer.Annotations.Signal;
import frc.robot.Robot;

/**
 * Our favorite ADXRS450 and 453 gyro doesn't yet fully support
 * all required features in sim, so we're adding this wrapper to
 * provide those features;
 */
public class WrapperedADXRS450 {

    ADXRS450_Gyro gyro;

    Rotation2d readingOffset = Rotation2d.fromDegrees(0);

    @Signal(units="deg")
    double rawGyroAngle = 0;

    public WrapperedADXRS450(){
        gyro = new ADXRS450_Gyro();
        
    } 

    public void resetToAngle(Rotation2d angle){
        if(Robot.isSimulation()){
            readingOffset = Rotation2d.fromDegrees(gyro.getAngle()).minus(angle);
        } else {
            gyro.reset();
        }
    }

    public double getRate(){
        return gyro.getRate();
    }

    public Rotation2d getAngle(){
        rawGyroAngle = gyro.getAngle();
        return Rotation2d.fromDegrees(rawGyroAngle).plus(readingOffset);
    }

    public void calibrate(){
        gyro.calibrate();
    }

    public boolean isConnected(){
        return gyro.isConnected();
    }


}