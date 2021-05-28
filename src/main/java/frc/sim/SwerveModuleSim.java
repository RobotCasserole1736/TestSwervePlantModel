package frc.sim;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import frc.sim.physics.Vector2d;

public class SwerveModuleSim {

    private DCMotor azimuthMotor;
    private DCMotor wheelMotor;
    private EncoderSim azimuthEncoder;
    private EncoderSim wheelEncoder;

    private double wheelRadius_m;
    private double azimuthGearRatio;       //Motor-to-azimuth reduction
    private double wheelGearRatio;         //Motor-to-wheel reduction
    private double azimuthEncGearRatio;    //Motor-to-azimuth-encoder reduction
    private double wheelEncGearRatio;      //Motor-to-wheel-encoder reduction
    private double treadStaticCoefFric;
    private double treadKineticCoefFric;
    private double azimuthEffectiveMOI;
    private double wheelEffectiveMOI;

    Pose2d prevModulePose = null;
    Pose2d curModulePose  = null;
    double curLinearSpeed_mps = 0; //Positive = in curAngle_deg, Negative = opposite of curAngle_deg
    Rotation2d curAzmthAngle = Rotation2d.fromDegrees(0); //0 = toward front, 90 = toward left, 180 = toward back, 270 = toward right

    double crossTreadFricForceMag = 0;
    double crossTreadVelMag = 0;
    double crossTreadForceMag = 0;

    public SwerveModuleSim(
        DCMotor azimuthMotor,
        DCMotor wheelMotor,
        EncoderSim azimuthEncoder, 
        EncoderSim wheelEncoder,   
        double wheelRadius_m,
        double azimuthGearRatio,      
        double wheelGearRatio,        
        double azimuthEncGearRatio,   
        double wheelEncGearRatio,     
        double treadStaticCoefFric,
        double treadKineticCoefFric,
        double azimuthEffectiveMOI,
        double wheelEffectiveMOI
    ){
        this.azimuthMotor = azimuthMotor;
        this.wheelMotor = wheelMotor;
        this.azimuthEncoder = azimuthEncoder;
        this.wheelEncoder = wheelEncoder;
        this.wheelRadius_m         = wheelRadius_m;
        this.azimuthGearRatio      = azimuthGearRatio;     
        this.wheelGearRatio        = wheelGearRatio;       
        this.azimuthEncGearRatio   = azimuthEncGearRatio; 
        this.wheelEncGearRatio     = wheelEncGearRatio;   
        this.treadStaticCoefFric   = treadStaticCoefFric;
        this.treadKineticCoefFric  = treadKineticCoefFric; 
        this.azimuthEffectiveMOI   = azimuthEffectiveMOI;
        this.wheelEffectiveMOI     = wheelEffectiveMOI;
    }

    public void reset(Pose2d initModulePose){
        prevModulePose = curModulePose = initModulePose;
        curLinearSpeed_mps = 0;
        curAzmthAngle = Rotation2d.fromDegrees(0);
    }

    
    public void update(double batteryVoltage){

        Vector2d azimuthUnitVec = new Vector2d(1,0);
        azimuthUnitVec.rotate(curAzmthAngle.getDegrees());

        // Assume the wheel does not lose traction along its wheel direction (on-tread)
        double velocityAlongAzimuth = getModuleRelativeTranslationVelocity().dot(azimuthUnitVec);

        wheelMotor.update(velocityAlongAzimuth, batteryVoltage_v, wheelCmd);
        azmthMotor.update(batteryVoltage_v, angleCmd);

        // Assume idealized azimuth control - no "twist" force at contact patch from friction or robot motion.
        curAzmthAngle = Rotation2d.fromDegrees(azmthMotor.getPosition_Rev() * 360);

        wheelEncoder.setShaftPositionRev(0); //TODO
        azimuthEncoder.setShaftPositionRev(0); //TODO
    }


}
