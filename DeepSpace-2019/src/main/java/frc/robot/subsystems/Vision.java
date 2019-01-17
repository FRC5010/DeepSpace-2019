/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Add your docs here.
 */
public class Vision extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  NetworkTable table;
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;

  double steerKp = 0.023;
  double moveKp = 0.15;
  double moveMin = 0.19;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void updateTable() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
  }

  public double getX() {
    updateTable();
    return table.getEntry("tx").getDouble(0.0);
  }

  public double getY() {
    updateTable();
    return table.getEntry("ty").getDouble(0.0);
  }

  public double getA() {
    updateTable();
    return table.getEntry("ta").getDouble(0.0);
  }

  //returns a steering motor output to turn robot towards target
  public double turnTowardsTarget() {
    double steerAmt = steerKp * getX();

    if (getX() > 1.0) {
      steerAmt += moveMin;
    } else if (getX() < 1.0) {
      steerAmt -= moveMin;
    }

    return -steerAmt;
  }

  public double moveTowardsTarget() {
    double moveAmt = 0;

    if (getX() != 0) {
      moveAmt = moveKp * (10.0 - -getY());
      return -moveAmt;
    }
    return 0;
  }

}
