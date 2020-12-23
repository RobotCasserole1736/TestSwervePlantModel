/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.sim.physics;

import java.util.Objects;

import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;

public class ForceAtDistance2d {
  public Force2d force;
  public Transform2d pos;

  /**
   * Constructs a ForceAtDistance2d that's all zeroed out
   */
  public ForceAtDistance2d () {
    this(new Force2d(), new Transform2d());
  }

  /**
   * Constructs a Force2d with the X and Y components equal to the
   * provided values.
   *
   * @param x The x component of the force.
   * @param y The y component of the force.
   */
  public ForceAtDistance2d ( Force2d force_in, Transform2d pos_in) {
    force = force_in;
    pos = pos_in;
  }

  /** 
   * Returns the torque associated with this force at distance
   * positive is counter-clockwise, negative is clockwise
   */
  public double getTorque(){
    Vector2d leverArm = force.getUnitVector();
    leverArm.rotate(90);
    return force.vec.dot(leverArm);
  }

  @Override
  public String toString() {
    return String.format("Force2d(X: %.2f, Y: %.2f)", vec.x, vec.y);
  }

  /**
   * Checks equality between this Force2d and another object.
   *
   * @param obj The other object.
   * @return Whether the two objects are equal or not.
   */
  @Override
  public boolean equals(Object obj) {
    if (obj instanceof Force2d) {
      return Math.abs(((Force2d) obj).vec.x - vec.x) < 1E-9
          && Math.abs(((Force2d) obj).vec.y - vec.y) < 1E-9;
    }
    return false;
  }

  @Override
  public int hashCode() {
    return Objects.hash(vec.x, vec.y);
  }
}
