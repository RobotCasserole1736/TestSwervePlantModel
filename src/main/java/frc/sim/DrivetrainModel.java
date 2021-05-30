package frc.sim;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Units;
import frc.Constants;
import frc.PoseTelemetry;
import frc.sim.wpiClasses.QuadSwerveSim;
import frc.sim.wpiClasses.SwerveModuleSim;

class DrivetrainModel {


    QuadSwerveSim swerveDt;
    ArrayList<SwerveModuleSim> modules = new ArrayList<SwerveModuleSim>(QuadSwerveSim.NUM_MODULES);

    ArrayList<PWMSim> azmthMotorControllers = new ArrayList<PWMSim>(QuadSwerveSim.NUM_MODULES);
    ArrayList<PWMSim> wheelMotorControllers = new ArrayList<PWMSim>(QuadSwerveSim.NUM_MODULES);
    ArrayList<SimQuadratureEncoder> azmthEncoders = new ArrayList<SimQuadratureEncoder>(QuadSwerveSim.NUM_MODULES);
    ArrayList<SimQuadratureEncoder> wheelEncoders = new ArrayList<SimQuadratureEncoder>(QuadSwerveSim.NUM_MODULES);


    SimGyroSensorModel gyro;
    SimVisionModel vision;

    Field2d field;
    Pose2d endPose;

    static SwerveModuleSim swerveModuleFactory(){
        return new SwerveModuleSim(DCMotor.getNEO(1), 
                                   DCMotor.getNEO(1), 
                                   Units.inchesToMeters(Constants.WHEEL_RADIUS_IN),
                                   12.0,
                                   25.0,
                                   1.0,
                                   1.0,
                                   1.1,
                                   0.8,
                                   Constants.ROBOT_MASS_kg * 9.81 / QuadSwerveSim.NUM_MODULES, 
                                   1  //TODO - look up
                                   );
    }

    public DrivetrainModel(){

        modules.add(swerveModuleFactory()); //FL
        modules.add(swerveModuleFactory()); //FR
        modules.add(swerveModuleFactory()); //BL
        modules.add(swerveModuleFactory()); //BR

        azmthMotorControllers.add(new PWMSim(Constants.FL_AZMTH_MOTOR_IDX));
        azmthMotorControllers.add(new PWMSim(Constants.FR_AZMTH_MOTOR_IDX));
        azmthMotorControllers.add(new PWMSim(Constants.BL_AZMTH_MOTOR_IDX));
        azmthMotorControllers.add(new PWMSim(Constants.BR_AZMTH_MOTOR_IDX));

        wheelMotorControllers.add(new PWMSim(Constants.FL_WHEEL_MOTOR_IDX));
        wheelMotorControllers.add(new PWMSim(Constants.FR_WHEEL_MOTOR_IDX));
        wheelMotorControllers.add(new PWMSim(Constants.BL_WHEEL_MOTOR_IDX));
        wheelMotorControllers.add(new PWMSim(Constants.BR_WHEEL_MOTOR_IDX));

        azmthEncoders.add(new SimQuadratureEncoder(Constants.FL_AZMTH_ENC_A_IDX, Constants.ENC_PULSE_PER_REV, Constants.AZMTH_ENC_MODULE_REVS_PER_COUNT));
        azmthEncoders.add(new SimQuadratureEncoder(Constants.FR_AZMTH_ENC_A_IDX, Constants.ENC_PULSE_PER_REV, Constants.AZMTH_ENC_MODULE_REVS_PER_COUNT));
        azmthEncoders.add(new SimQuadratureEncoder(Constants.BL_AZMTH_ENC_A_IDX, Constants.ENC_PULSE_PER_REV, Constants.AZMTH_ENC_MODULE_REVS_PER_COUNT));
        azmthEncoders.add(new SimQuadratureEncoder(Constants.BR_AZMTH_ENC_A_IDX, Constants.ENC_PULSE_PER_REV, Constants.AZMTH_ENC_MODULE_REVS_PER_COUNT));

        wheelEncoders.add(new SimQuadratureEncoder(Constants.FL_WHEEL_ENC_A_IDX, Constants.ENC_PULSE_PER_REV, Constants.WHEEL_ENC_WHEEL_REVS_PER_COUNT));
        wheelEncoders.add(new SimQuadratureEncoder(Constants.FR_WHEEL_ENC_A_IDX, Constants.ENC_PULSE_PER_REV, Constants.WHEEL_ENC_WHEEL_REVS_PER_COUNT));
        wheelEncoders.add(new SimQuadratureEncoder(Constants.BL_WHEEL_ENC_A_IDX, Constants.ENC_PULSE_PER_REV, Constants.WHEEL_ENC_WHEEL_REVS_PER_COUNT));
        wheelEncoders.add(new SimQuadratureEncoder(Constants.BR_WHEEL_ENC_A_IDX, Constants.ENC_PULSE_PER_REV, Constants.WHEEL_ENC_WHEEL_REVS_PER_COUNT));

        gyro = new SimGyroSensorModel();
        vision = new SimVisionModel();

        field = PoseTelemetry.field;
        field.setRobotPose(Constants.DFLT_START_POSE);
        endPose = Constants.DFLT_START_POSE;

        swerveDt = new QuadSwerveSim(Constants.WHEEL_BASE_WIDTH_M, 
                                     Constants.WHEEL_BASE_WIDTH_M, 
                                     Constants.ROBOT_MASS_kg, 
                                     Constants.ROBOT_MOI_KGM2, 
                                     modules);
    }

    /**
     * Handles discontinuous jumps in robot pose. Used at the start of
     * autonomous, if the user manually drags the robot across the field in the
     * Field2d widget, or something similar to that.
     * @param pose
     */
    public void modelReset(Pose2d pose){
        field.setRobotPose(pose);
        swerveDt.modelReset(pose);
        gyro.resetToPose(pose);
    }

    /**
     * Advance the simulation forward by one step
     * @param isDisabled
     * @param batteryVoltage
     */
    public void update(boolean isDisabled, double batteryVoltage){
 
        // Check if the user moved the robot with the Field2D
        // widget, and reset the model if so.
        Pose2d startPose = field.getRobotPose();
        if(!endPose.equals(startPose)){
            modelReset(startPose);
        }

        // Calculate and update input voltages to each motor.
        if(isDisabled){
            for(int idx = 0; idx < QuadSwerveSim.NUM_MODULES; idx++){
                modules.get(idx).setInputVoltages(0.0, 0.0);
            }
        } else {
            for(int idx = 0; idx < QuadSwerveSim.NUM_MODULES; idx++){
                double azmthVolts = azmthMotorControllers.get(idx).getSpeed() * batteryVoltage;
                double wheelVolts = wheelMotorControllers.get(idx).getSpeed() * batteryVoltage;
                modules.get(idx).setInputVoltages(wheelVolts, azmthVolts);
            }
        }

        //Update the main drivetrain plant model
        swerveDt.update(Constants.SIM_SAMPLE_RATE_SEC);
        endPose = swerveDt.getCurPose();

        // Update each encoder
        for(int idx = 0; idx < QuadSwerveSim.NUM_MODULES; idx++){
            Rotation2d azmthPos = modules.get(idx).getAzimuthEncoderPosition();
            Rotation2d wheelPos = modules.get(idx).getWheelEncoderPosition();
            azmthEncoders.get(idx).setShaftPosition(azmthPos, Constants.SIM_SAMPLE_RATE_SEC);
            wheelEncoders.get(idx).setShaftPosition(wheelPos, Constants.SIM_SAMPLE_RATE_SEC);
        }

        // Update associated devices based on drivetrain motion
        field.setRobotPose(endPose);
        gyro.update(startPose, endPose);
        vision.update(endPose);

        endPose = startPose;
    }

    public double getCurrentDraw(){
        double retVal = 0;
        for(int idx = 0; idx < Constants.NUM_MODULES; idx++){
            retVal += modules.get(idx).getCurrentDraw_A();
        }
        return retVal;
    }

}