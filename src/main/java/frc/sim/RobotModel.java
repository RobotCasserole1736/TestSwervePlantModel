package frc.sim;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.simulation.PDPSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class RobotModel {

    private DrivetrainModel dt;

    PDPSim pdp;

    final double QUIESCENT_CURRENT_DRAW_A = 2.0; //Misc electronics
    final double BATTERY_NOMINAL_VOLTAGE = 13.2; //Nicely charged battery
    final double BATTERY_NOMINAL_RESISTANCE = 0.040; //40mOhm - average battery + cabling

    double currentDraw_A = QUIESCENT_CURRENT_DRAW_A;
    double batteryVoltage_V = BATTERY_NOMINAL_VOLTAGE;
    boolean isBrownedOut;


    public RobotModel(){
        dt = new DrivetrainModel();
        pdp = new PDPSim();
        reset();
    }

    public void reset(){
        dt.modelReset();
    }

    public void update(boolean isDisabled){

        //Calculate motor disablement due to either actually being in disabled mode,
        // or due to brownout.
        isBrownedOut = (batteryVoltage_V < 6.5);
        isDisabled |= isBrownedOut;

        dt.update(isDisabled, batteryVoltage_V);

        currentDraw_A = QUIESCENT_CURRENT_DRAW_A + dt.getCurrentDraw();

        // Temp - beta_3 has a flipped argument in the flywheel sim system which causes the current calculation to be incorrect.
        //batteryVoltage_V = BatterySim.calculateLoadedBatteryVoltage(BATTERY_NOMINAL_VOLTAGE, BATTERY_NOMINAL_RESISTANCE, currentDraw_A);

        RoboRioSim.setVInVoltage(batteryVoltage_V);
        pdp.setVoltage(batteryVoltage_V);
        pdp.setCurrent(0,currentDraw_A); //Hack just so that getTotalCurrent works in robot code
        

    }

    public Pose2d getCurActPose(){
        return dt.field.getRobotObject().getPose();
    }

}