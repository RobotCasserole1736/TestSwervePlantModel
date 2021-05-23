package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.DataServer.Annotations.Signal;

public class LoopTiming{

    //All times in seconds
    double loopStartTime;
    double loopEndTime;

    double prevLoopStartTime;
    double prevLoopEndTime;

    @Signal(units = "sec")
    double loopPeriodSec;

    @Signal(units = "sec")
    double loopDurationSec;

    /* Singleton stuff */
    private static LoopTiming loopTiming = null;
    public static synchronized LoopTiming getInstance() {
        if(loopTiming == null)
            loopTiming = new LoopTiming();
        return loopTiming;
    }

    private LoopTiming(){

    }

    public void markLoopStart(){
        prevLoopStartTime = loopStartTime;
        loopStartTime = Timer.getFPGATimestamp();
        loopPeriodSec = loopStartTime - prevLoopStartTime;
    }

    public void markLoopEnd(){
        prevLoopEndTime = loopEndTime;
        loopEndTime = Timer.getFPGATimestamp();
        loopDurationSec = loopEndTime - loopStartTime;
    }

    public double getLoopStartTimeSec(){
        return loopStartTime;
    }

    public double getPeriodSec(){
        return loopPeriodSec;
    }
}