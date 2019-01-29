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
  
  class GearVars {
    public final double steerKp, moveKp, moveMin;
    public GearVars (double psteerKp, double pmoveKp, double pmoveMin) {
      steerKp = psteerKp;
      moveKp = pmoveKp;
      moveMin = pmoveMin;
    }
  }
  
  // two different sets of Kp values for different gears
  GearVars lowGear = new GearVars(0.02, 0.015, 0.07);
  GearVars highGear = new GearVars(0.02, 0.015, 0.07);

  double desiredDistance = 40;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  // returns a steering motor output to turn robot towards target
  public double turnTowardsTarget() {
    double steerAmt = 0;
    if (Pose.getCurrentPose().limeLightValid) {
      steerAmt = (Shifter.isLowGear ? lowGear.steerKp : highGear.steerKp) * Pose.getCurrentPose().limeLightTx;

      if (Pose.getCurrentPose().limeLightTx > 1.0) {
        steerAmt += Shifter.isLowGear ? lowGear.moveMin : highGear.moveMin;
      } else if (Pose.getCurrentPose().limeLightTx < 1.0) {
        steerAmt -= Shifter.isLowGear ? lowGear.moveMin : highGear.moveMin;
      }
    }
    return -steerAmt;
  }

  // returns a motor output that will move the robot towards target
  public double moveTowardsTarget() {
    double moveAmt = 0;

    double distanceFromTarget = Pose.getCurrentPose().limeLightDistance;
    if (Pose.getCurrentPose().limeLightValid) {
      double error = desiredDistance - distanceFromTarget;
      moveAmt = (Shifter.isLowGear ? lowGear.moveKp : highGear.moveKp) * error;

      if (error > 1.0) {
        moveAmt += Shifter.isLowGear ? lowGear.moveMin : highGear.moveMin;
      } else if (error < 1.0) {
        moveAmt -= Shifter.isLowGear ? lowGear.moveMin : highGear.moveMin;
      }

      return moveAmt;
    }
    return 0;
  }

  /* public double arcTowardsTarget() {
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
  } */
}
