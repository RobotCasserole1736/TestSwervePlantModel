
package frc.robot;

import edu.wpi.first.wpilibj.Spark;

class TestDtCtrl {

    Spark FLWheelCtrl;
    Spark FLAngleCtrl;
    Spark FRWheelCtrl;
    Spark FRAngleCtrl;
    Spark BLWheelCtrl;
    Spark BLAngleCtrl;
    Spark BRWheelCtrl;
    Spark BRAngleCtrl;

    public TestDtCtrl(){
        FLWheelCtrl = new Spark(0);
        FLAngleCtrl = new Spark(1);
        FRWheelCtrl = new Spark(2);
        FRAngleCtrl = new Spark(3);
        BLWheelCtrl = new Spark(4);
        BLAngleCtrl = new Spark(5);
        BRWheelCtrl = new Spark(6);
        BRAngleCtrl = new Spark(7);   
            
    }

    public void update(){

        FLWheelCtrl.set(0.25);
        FRWheelCtrl.set(0.25);
        BLWheelCtrl.set(0.25);
        BRWheelCtrl.set(0.25);

        FLAngleCtrl.set(0.1);
        FRAngleCtrl.set(-0.1);
        BLAngleCtrl.set(-0.1);
        BRAngleCtrl.set(0.1);

    }
}