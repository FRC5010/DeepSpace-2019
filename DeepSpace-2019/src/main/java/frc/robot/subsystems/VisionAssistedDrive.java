/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class VisionAssistedDrive extends Subsystem {

  public static class GearVariables {
    public final double steerKp, moveKp, moveMin;

    public GearVariables(double psteerKp, double pmoveKp, double pmoveMin) {
      steerKp = psteerKp;
      moveKp = pmoveKp;
      moveMin = pmoveMin;
    }
  }

  // two different sets of Kp values for different gears
  public static GearVariables lowGear = new GearVariables(0.0075, 0.08, 0.07);
  public static GearVariables highGear = new GearVariables(0.01, 0.08, 0.07);

  double minRotationDistance = 40;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  // returns a steering motor output to turn robot towards target
  public double turnTowardsTarget() {
    double steerAmt = 0;
    if (Pose.getCurrentPose().limeLight.tValid) {
      steerAmt = (Shifter.isLowGear ? lowGear.steerKp : highGear.steerKp) * Pose.getCurrentPose().limeLight.tX;
      
      if (Pose.getCurrentPose().limeLight.tX > 1.0) {
        steerAmt += Shifter.isLowGear ? lowGear.moveMin : highGear.moveMin;
      } else if (Pose.getCurrentPose().limeLight.tX < 1.0) {
        steerAmt -= Shifter.isLowGear ? lowGear.moveMin : highGear.moveMin;
      }
    }
    return steerAmt;
  }

  public static double moveTowardsTarget() {
    double moveAmt = 0;
    if (Pose.getCurrentPose().limeLight.tValid) {
      double tY = Pose.getCurrentPose().limeLight.tY;
      moveAmt = (Shifter.isLowGear ? lowGear.moveKp : highGear.moveKp) * tY;

      double moveMin = Shifter.isLowGear ? lowGear.moveMin : highGear.moveMin;
      moveAmt = Math.max(moveMin, Math.abs(moveAmt)) * Math.signum(moveAmt);
      // if (tY < 1.0) {
      //   moveAmt += Shifter.isLowGear ? lowGear.moveMin : highGear.moveMin;
      // } else if (Math.signum(tY) == -1 && tY > -1.0) {
      //   moveAmt -= Shifter.isLowGear ? lowGear.moveMin : highGear.moveMin;
      // }
    }
    return moveAmt;
  }

  // returns a motor output that will move the robot towards target
  /*
  public double moveTowardsTarget() {
    double moveAmt = 0;

    double distanceFromTarget = Pose.getCurrentPose().limeLight.tDistance;
    if (Pose.getCurrentPose().limeLight.tValid) {
      double error = minRotationDistance - distanceFromTarget;
      moveAmt = (Shifter.isLowGear ? lowGear.moveKp : highGear.moveKp) * error;

      if (error > 1.0) {
        moveAmt += Shifter.isLowGear ? lowGear.moveMin : highGear.moveMin;
      } else if (error < 1.0) {
        moveAmt -= Shifter.isLowGear ? lowGear.moveMin : highGear.moveMin;
      }

      return -moveAmt;
    }
    return 0;
  }
  */

  // arc towards target
  public static double arcTowardsTarget() {
    Pose currentPose = Pose.getCurrentPose();
    if (currentPose.limeLight.tValid) {
      double distance = currentPose.limeLight.tDistance;
      System.out.println("-------");
      double rotationAngle = currentPose.limeLight.tX;
      System.out.println("Angle: " + rotationAngle);
      double errorDistance = distance; // - minRotationDistance;
      System.out.println("ErrorDistance: " + errorDistance);
      double approachAngle = /*Math.signum(currentPose.limeLight.aspectApproachAngle) */ 
        (errorDistance > 0 ? errorDistance / 3.0 : 0.0);
      System.out.println("DesAngle: " + approachAngle);
      
      double error = /*(currentPose.limeLight.aspectApproachAngle < approachAngle) ? 0.0 :*/ approachAngle - rotationAngle;
      System.out.println("Error: " + error);
      double steerAmt = (Shifter.isLowGear ? lowGear.steerKp : highGear.steerKp) * error;
      System.out.println("SteerAmt: " + steerAmt);

      double moveMin = Shifter.isLowGear ? lowGear.moveMin : highGear.moveMin;
      steerAmt = Math.signum(steerAmt) * Math.max(Math.abs(steerAmt), moveMin);

      return -steerAmt;
    }
    return 0.0;
  }
}
