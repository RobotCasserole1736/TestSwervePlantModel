package frc.sim;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.EncoderSim;

public class SimQuadratureEncoder{

    EncoderSim baseEnc;

    Rotation2d shaftPosPrev = new Rotation2d();
    long curCyclCount = 0;

    final int CYCLES_PER_REV;
    final double DIST_PER_CYCLE;

    public SimQuadratureEncoder(int pinA, int cyclesPerRev_in, double dist_per_pulse){
        baseEnc = EncoderSim.createForChannel(pinA);
        DIST_PER_CYCLE = dist_per_pulse; //TODO - we really ought to be able to get this from user-space configuration of the baseEncoder, but can't currently?
        CYCLES_PER_REV = cyclesPerRev_in;

    }

    public void reset(){
        shaftPosPrev = new Rotation2d();
        curCyclCount = 0;
    }


    public void setShaftPosition(Rotation2d shaftPos, double dtSeconds){
        
        //Fairly simple model of encoder internals & FPGA interfacing? Maybe?
        double deltaDegrees = (shaftPos.minus(shaftPosPrev).getDegrees()) * (baseEnc.getReverseDirection() ? -1.0 : 1.0);
        double deltaCycles = Math.floor(CYCLES_PER_REV * deltaDegrees * 360.0);

        curCyclCount += deltaCycles;

        double distance = curCyclCount * DIST_PER_CYCLE;
        double rate = deltaCycles / dtSeconds * DIST_PER_CYCLE ;
        boolean direction = (rate < 0);


        baseEnc.setCount((int) Math.floor(curCyclCount));
        baseEnc.setDistance(distance);
        baseEnc.setRate(rate);
        baseEnc.setDirection(direction);

        shaftPosPrev = shaftPos;
    }
}