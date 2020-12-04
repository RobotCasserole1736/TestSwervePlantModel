package frc.sim;

import edu.wpi.first.hal.sim.PWMSim;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Twist2d;
import frc.lib.DataServer.Signal;

class DrivetrainModel {
    PWMSim lhMotorCtrl1;
    PWMSim lhMotorCtrl2;
    PWMSim lhMotorCtrl3;
    PWMSim rhMotorCtrl1;
    PWMSim rhMotorCtrl2;
    PWMSim rhMotorCtrl3;

    final double SAMPLE_RATE_SEC = 0.02;

    final double DT_TRACK_WIDTH_FT = 2.5;
    final double DT_MAX_SPEED_FT_PER_SEC = 16.0;
    final double WHEEL_RADIUS_FT = 6.0/2.0/12.0; //six inch diameter wheels

    SimpleMotorWithMassModel lhSide;
    SimpleMotorWithMassModel rhSide;

    Signal xPosFtSig;
    Signal yPosFtSig;
    Signal tRotDegSig;

    Signal xPosDesFtSig;
    Signal yPosDesFtSig;
    Signal tRotDesDegSig;

    Field2d field;
    Pose2d dtPoseForTelemetry;

    final Pose2d START_POSE = new Pose2d(ftToM(0), ftToM(0), Rotation2d.fromDegrees(0));

    public DrivetrainModel(){
        lhMotorCtrl1 = new PWMSim(0);
        lhMotorCtrl2 = new PWMSim(1);
        lhMotorCtrl3 = new PWMSim(2);
        rhMotorCtrl1 = new PWMSim(3);
        rhMotorCtrl2 = new PWMSim(4);
        rhMotorCtrl3 = new PWMSim(5);
        lhSide = new SimpleMotorWithMassModel("DT Left",  MPerSectoRPM(ftToM(DT_MAX_SPEED_FT_PER_SEC)), 0.1, 250);
        rhSide = new SimpleMotorWithMassModel("DT Right", MPerSectoRPM(ftToM(DT_MAX_SPEED_FT_PER_SEC)), 0.1, 250);
        xPosFtSig     = new Signal("botActPoseX", "ft");
        yPosFtSig     = new Signal("botActPoseY", "ft");
        tRotDegSig    = new Signal("botActPoseT", "deg");
        xPosDesFtSig  = new Signal("botDesPoseX", "ft");
        yPosDesFtSig  = new Signal("botDesPoseY", "ft");
        tRotDesDegSig = new Signal("botDesPoseT", "deg");

        field = new Field2d();
        dtPoseForTelemetry = new Pose2d();
    }

    public void modelReset(){
        lhSide.modelReset();
        rhSide.modelReset();
        field.setRobotPose(START_POSE);
    }

    public void update(boolean isDisabled){

        Pose2d dtPos = field.getRobotPose();

        double lhMotorCmd  = 0;
        double rhMotorCmd  = 0;

        // Eeh, kinda. Current draw won't be accurate if the student accidentally boogers up the motor controller outputs.
        if(!isDisabled){
            lhMotorCmd = (lhMotorCtrl1.getSpeed() + lhMotorCtrl2.getSpeed() + lhMotorCtrl3.getSpeed())/3;
            rhMotorCmd = (rhMotorCtrl1.getSpeed() + rhMotorCtrl2.getSpeed() + rhMotorCtrl3.getSpeed())/3;
        }


        lhSide.update(12.5, lhMotorCmd, 0.0);
        rhSide.update(12.5, rhMotorCmd, 0.0);

        double lhWheelDeltaM = RPMtoMPerSec(lhSide.getSpeed_RPM())*SAMPLE_RATE_SEC;
        double rhWheelDeltaM = RPMtoMPerSec(rhSide.getSpeed_RPM())*SAMPLE_RATE_SEC;

        Twist2d dtTwist = new Twist2d((lhWheelDeltaM + rhWheelDeltaM)/2, 0, (lhWheelDeltaM - rhWheelDeltaM) / (ftToM(DT_TRACK_WIDTH_FT)));

        dtPos = dtPos.exp(dtTwist);

        field.setRobotPose(dtPos);

        dtPoseForTelemetry = dtPos;
    }

    public void updateTelemetry(double time){
        lhSide.updateTelemetry(time);
        rhSide.updateTelemetry(time);
        xPosFtSig.addSample(time,  mToFt(dtPoseForTelemetry.getTranslation().getX()));
        yPosFtSig.addSample(time,  mToFt(dtPoseForTelemetry.getTranslation().getY()));
        tRotDegSig.addSample(time, dtPoseForTelemetry.getRotation().getDegrees());
        xPosDesFtSig.addSample(time,  0);
        yPosDesFtSig.addSample(time,  0);
        tRotDesDegSig.addSample(time, 0);
    }

    double ftToM(double ft_in){ return ft_in * 0.3048; }
    double mToFt(double m_in){ return m_in / 0.3048; }
    double RPMtoMPerSec(double rot_spd_in){ return rot_spd_in * (1.0/60.0) * (2.0 * Math.PI * ftToM(WHEEL_RADIUS_FT)); }
    double MPerSectoRPM(double lin_spd_in){ return lin_spd_in / (1.0/60.0) / (2.0 * Math.PI * ftToM(WHEEL_RADIUS_FT)); }


}