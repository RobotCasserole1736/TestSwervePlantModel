
package frc.robot;

import edu.wpi.first.wpilibj.Spark;

class TestDtCtrl {

    Spark lhMotorCtrl1;
    Spark lhMotorCtrl2;
    Spark lhMotorCtrl3;
    Spark rhMotorCtrl1;
    Spark rhMotorCtrl2;
    Spark rhMotorCtrl3;

    public TestDtCtrl(){
        lhMotorCtrl1 = new Spark(0);
        lhMotorCtrl2 = new Spark(1);
        lhMotorCtrl3 = new Spark(2);
        rhMotorCtrl1 = new Spark(3);
        rhMotorCtrl2 = new Spark(4);
        rhMotorCtrl3 = new Spark(5);
    }

    public void update(){

        lhMotorCtrl1.set(0.25); // Temp - just move forward slightly
        lhMotorCtrl2.set(0.25); // Temp - just move forward slightly
        lhMotorCtrl3.set(0.25); // Temp - just move forward slightly
        rhMotorCtrl1.set(0.25); // Temp - just move forward slightly
        rhMotorCtrl2.set(0.25); // Temp - just move forward slightly
        rhMotorCtrl3.set(0.25); // Temp - just move forward slightly

    }
}