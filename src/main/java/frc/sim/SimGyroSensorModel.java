package frc.sim;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import frc.Constants;

public class SimGyroSensorModel{

    ADXRS450_GyroSim gyroSim;
    double gyroPosReading_deg;

    // Limit what the gyro itself can read
    // Ex for ADSRX453 - https://www.analog.com/media/en/technical-documentation/data-sheets/ADXRS453.pdf
    final double GYRO_RATE_SCALING_DEGPERSEC_PER_BIT = 1.0/80.0;
    final double GYRO_MAX_MEASURABLE_RATE_DEGPERSEC = 400.0;

    public SimGyroSensorModel(){
        gyroSim = new ADXRS450_GyroSim( new ADXRS450_Gyro()); //Use default gyro port and some new instance to not require tie to user code.

    }

    public void update(Pose2d curRobotPose, Pose2d prevRobotPose){

        double curGyroAngle  = curRobotPose.getRotation().getDegrees();
        double prevGyroAngle = prevRobotPose.getRotation().getDegrees();
        double gyroRate = -1.0 * (curGyroAngle - prevGyroAngle)/Constants.SIM_SAMPLE_RATE_SEC; //Gyro reads backward from sim reference frames.
        
        gyroRate = Math.min(gyroRate,  GYRO_MAX_MEASURABLE_RATE_DEGPERSEC);
        gyroRate = Math.max(gyroRate, -GYRO_MAX_MEASURABLE_RATE_DEGPERSEC);

        //Round gyro rate to gyro internal scaling
        long gyroBits = Math.round(gyroRate / GYRO_RATE_SCALING_DEGPERSEC_PER_BIT);

        gyroRate = gyroBits * GYRO_RATE_SCALING_DEGPERSEC_PER_BIT;
        
        //Simulate the SPI accumulator action
        gyroPosReading_deg += gyroRate * Constants.SIM_SAMPLE_RATE_SEC;

        // Pass our model of what the sensor would be measuring back into the simGyro object
        // for hte embedded code to interact with.
        gyroSim.setAngle(gyroPosReading_deg);
        gyroSim.setRate(gyroRate);


    }
}