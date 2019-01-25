/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * Add your docs here.
 */
public class VisionAssistedDrive extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  static final double steerKp = 0.02;
  static final double moveKp = 0.015;
  static final double moveMin = 0.07;
  static final double targetHeight = 24;
  double desiredDistance = 40;
  private static double prevDist = 0.0;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  //returns the distance from the vision target
  /* public double distanceFromTarget() {
    double distance = prevDist;

    if (Pose.getCurrentPose().limeLightValid) {
      double yInRadians = Math.toRadians(Pose.getCurrentPose().limeLightTy);
      distance = Math.abs((Vision.LIME_LIGHT_HEIGHT - targetHeight) / Math.tan(yInRadians));
      SmartDashboard.putNumber("TanY", Math.tan(yInRadians));
      prevDist = (prevDist + distance) / 2;
      distance = prevDist;
    } else {
      distance = Double.NaN;
      prevDist = 0.0;
    }
    SmartDashboard.putNumber("DistanceFromTarget", distance);
    return distance;
  } */

  //returns a steering motor output to turn robot towards target
  public double turnTowardsTarget() {
    double steerAmt = 0;
    if (Pose.getCurrentPose().limeLightValid) {
      steerAmt = steerKp * Pose.getCurrentPose().limeLightTx;

      if (Pose.getCurrentPose().limeLightTx > 1.0) {
        steerAmt += moveMin;
      } else if (Pose.getCurrentPose().limeLightTx < 1.0) {
        steerAmt -= moveMin;
      }
    }
    return -steerAmt;
  }

  //returns a motor output that will move the robot towards target
  public double moveTowardsTarget() {
    double moveAmt = 0;

    double distanceFromTarget = Pose.getCurrentPose().limeLightDistance;
    if (Pose.getCurrentPose().limeLightValid) {
      double error = desiredDistance - distanceFromTarget;
      moveAmt = moveKp * error;

      if (error > 1.0) {
        moveAmt += moveMin;
      } else if (error < 1.0) {
        moveAmt -= moveMin;
      }

      return moveAmt;
    }
    return 0;
  }

  public double arcTowardsTarget() {
    double distance = Pose.getCurrentPose().limeLightDistance;
    if (Pose.getCurrentPose().limeLightValid) {
      System.out.println("-------");
      double angle = Pose.getCurrentPose().limeLightTx;
      System.out.println("Angle: " + angle);
      double calcDistance = distance - desiredDistance;
      System.out.println("CalcDist: " + calcDistance);
      double desiredAngle = (calcDistance > 0) ? calcDistance / 3.0 : 0.0;
      System.out.println("DesAngle: " + desiredAngle);
      double error = desiredAngle - Math.abs(angle);
      System.out.println("Error: " + error);
      double steerAmt = steerKp * error * (angle / Math.abs(angle));
      System.out.println("SteerAmt: " + steerAmt);

      if (steerAmt < moveMin && steerAmt > 0) {
        steerAmt = moveMin;
      } else if (steerAmt > -moveMin && steerAmt < 0) {
        steerAmt = -moveMin;
      }
      return -steerAmt;
    }
    return 0.0;
  }

}
