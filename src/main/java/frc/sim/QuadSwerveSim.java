package frc.sim;

import java.util.ArrayList;

public class QuadSwerveSim {

    ArrayList<SwerveModuleSim> modules = new ArrayList<SwerveModuleSim>(4);


    public QuadSwerveSim(
    double wheelBaseWidth_m,
    double weelBaseLength_m,
    double robotMass_kg,
    double robotMOI,
    SwerveModuleSim moduleFL,
    SwerveModuleSim moduleFR,
    SwerveModuleSim moduleBL,
    SwerveModuleSim moduleBR
    ){
        modules.set(1, moduleFL);
        modules.set(1, moduleFR);
        modules.set(1, moduleBL);
        modules.set(1, moduleBR);
        
    }
}
