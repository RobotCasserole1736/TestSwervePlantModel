/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.sim.physics;

/**
 * This is a 2D vector struct that supports basic vector operations.
 */
@SuppressWarnings("MemberName")
public class Vector2d extends edu.wpi.first.wpilibj.drive.Vector2d {

    public Vector2d(){
        super(0.0,0.0);
    }

    public Vector2d(double x, double y){
        super(x,y);
    }

    public double cross(Vector2d other){
        return this.x*other.y - this.y*other.x;
    }
}
