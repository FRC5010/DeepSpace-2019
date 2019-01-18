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
  double moveKp = 0.015;
  double moveMin = 0.07;

  double limelightHeight = 36;
  double targetHeight = 24;
  double desiredDistance = 60;

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

  public double distanceFromTarget() {
    double distance = 0;

    if (getX() != 0) {
      double yInRadians = Math.toRadians(getY());
      distance = (limelightHeight - targetHeight) / Math.tan(yInRadians);
      return Math.abs(distance);
    }

    return 0;
  }

  public double moveTowardsTarget() {
    double moveAmt = 0;

    if (getX() != 0) {
      double error = distanceFromTarget() - desiredDistance;
      moveAmt = moveKp * error;

      if (error > 1.0) {
        moveAmt += moveMin;
      } else if (error < 1.0) {
        moveAmt -= moveMin;
      }

      return -moveAmt;
    }
    return 0;
  }

  public double arcTowardsTarget() {
    double distance = distanceFromTarget();
    double angle = getX();
    double error = (distance / 3.0) - Math.abs(angle);
    double steerAmt = steerKp * error * (angle / Math.abs(angle));
    System.out.println("Error: " + error + ", Angle: " + angle + ", Distance/3: " + (distance / 3) + ", steerAmt: " + steerAmt);

    if (error > 1.0) {
      steerAmt += moveMin;
    } else if (error < 1.0) {
      steerAmt -= moveMin;
    }
    return -steerAmt;
  }

}
