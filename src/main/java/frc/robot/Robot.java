/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.Constants;
import frc.lib.Calibration.CalWrangler;
import frc.lib.DataServer.CasseroleDataServer;
import frc.lib.DataServer.Annotations.Signal;
import frc.lib.LoadMon.CasseroleRIOLoadMonitor;
import frc.lib.WebServer.CasseroleWebServer;
import frc.sim.RobotModel;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  DrivetrainControl dt;
  DrivetrainPoseEstimator dtpe;
  DrivetrainPathPlanner dtpp;

  DtPoseView dtPoseView;

  // Website utilities
  CasseroleWebServer webserver;
  CalWrangler wrangler;
  CasseroleDataServer dataServer;
  LoopTiming loopTiming;
  CasseroleRIOLoadMonitor loadMon;

  @Signal
  int loopCounter = 0;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {

    /* Init website utilties */
    webserver = new CasseroleWebServer();
    wrangler = new CalWrangler();
    dataServer = CasseroleDataServer.getInstance();
    loadMon = new CasseroleRIOLoadMonitor();

    dtPoseView = new DtPoseView();

    dt = new DrivetrainControl();
    dtpe = new DrivetrainPoseEstimator(dt);
    dtpp = new DrivetrainPathPlanner();


    if(isSimulation()){
      simModel = new RobotModel();
    }

    dataServer.registerSignals(this);
    dataServer.startServer();
    webserver.startServer();
  }

  /**
   * This function is called once as the robot enters autnonmous mode.
   */
  @Override
  public void autonomousInit() {

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    LoopTiming.getInstance().markLoopStart();
    periodicCommon();
    LoopTiming.getInstance().markLoopEnd();
  }

  /**
   * This function is called once as the robot enters teleop mode
   */
  @Override
  public void teleopInit() {
    dataServer.logger.startLoggingTeleop();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    LoopTiming.getInstance().markLoopStart();
    periodicCommon();
    LoopTiming.getInstance().markLoopEnd();
  }

  /**
   * This function is called once as the robot enters disabled mode
   */
  @Override
  public void disabledInit() {
    dataServer.logger.stopLogging();

    if(isSimulation()){
      simModel.reset();
      dtpe.setKnownPose(Constants.START_POSE);
    }
  }

  /**
   * This function is called periodically when disabled.
   */
  @Override
  public void disabledPeriodic() {
    LoopTiming.getInstance().markLoopStart();
    periodicCommon();
    LoopTiming.getInstance().markLoopEnd();
  }


  void periodicCommon() {
    loopCounter++;
    dt.update();
    dtpe.update();

    updateTelemetry();

  }

  private void updateTelemetry(){
    
    if(isSimulation()){
      dtPoseView.setActualPose(simModel.getCurActPose());
    }
    dtPoseView.setEstimatedPose(dtpe.getEstPose());
    dtPoseView.setDesiredPose(dtpp.getCurDesiredPose());

    double sampleTimeMs = LoopTiming.getInstance().getLoopStartTimeSec()*1000;
    dtPoseView.update(sampleTimeMs);
    dataServer.sampleAllSignals(sampleTimeMs);

  }


  /*=========================================================================*/
  /*=========================================================================*/

  /*
   * This set of functions is for simulation only, and is not called on the real
   * robot. Put plant-model related functionality here. For training purposes,
   * students should not have to modify this functionality.
   */

  // Simple robot plant model for simulation purposes
  RobotModel simModel;

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {

    simModel.update(isDisabled());
  }

  /*=========================================================================*/
  /*=========================================================================*/

}
