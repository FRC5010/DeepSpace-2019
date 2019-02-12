/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Pose;

public class TeleopDefault extends Command {

  private double moveAmt;
  private double steerAmt;
  private double deadZone = 0.15;

  public TeleopDefault() {
    requires(RobotMap.driveTrain);
  }

  public double scaleInputs(double input) {
    if (Math.abs(input) < deadZone) {
      input = 0;
    } else if (input > 0) {
      input = (input - deadZone) * 1 / (1 - deadZone);
    } else if (input < 0) {
      input = (input + deadZone) * 1 / (1 - deadZone);
    }

    return Math.pow(input, 3);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // System.out.println("Distance: " + Pose.getCurrentPose().limeLightDistance);

    if (scaleInputs(Robot.oi.driver.getRawAxis(3)) >= 0 && Pose.getCurrentPose().limeLight.tValid) {
      RobotMap.vision.changePipeline(Robot.oi.driverRB.get() ? 0 : 1);
      steerAmt = RobotMap.visionDrive.turnTowardsTarget();
      moveAmt = RobotMap.visionDrive.moveTowardsTarget();
      // steerAmt = RobotMap.visionDrive.arcTowardsTarget();
      RobotMap.driveTrain.drive(steerAmt + moveAmt, -steerAmt + moveAmt);
    } else {
      RobotMap.vision.changePipeline(0);
      moveAmt = -scaleInputs(Robot.oi.driver.getRawAxis(1));
      steerAmt = scaleInputs(Robot.oi.driver.getRawAxis(4));
      RobotMap.driveTrain.drive(moveAmt + steerAmt, moveAmt - steerAmt);
    }

    if (Robot.oi.driverStart.get()) {
      RobotMap.vision.toggleLimelight();
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
