
package frc.robot;

class DrivetrainControl {

    SwerveModuleControl moduleFL;
    SwerveModuleControl moduleFR;
    SwerveModuleControl moduleBL;
    SwerveModuleControl moduleBR;

    public DrivetrainControl(){
        moduleFL = new SwerveModuleControl("FL", 0,1,0,2);
        moduleFR = new SwerveModuleControl("FR", 2,3,4,6);
        moduleBL = new SwerveModuleControl("BL", 4,5,8,10);
        moduleBR = new SwerveModuleControl("BR", 6,7,12,14);
            
    }

    public void update(){

        moduleFL.update();
        moduleFR.update();
        moduleBL.update();
        moduleBR.update();
    }
}