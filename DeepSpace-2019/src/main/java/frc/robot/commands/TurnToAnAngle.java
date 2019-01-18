/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.RobotMap;

public class TurnToAnAngle extends Command {
  public TurnToAnAngle() {
    requires(RobotMap.driveTrain);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }
  double moveMin = 0.19;
  double steerKp = 0.023;
  Gyro gyro;

  public double turnTowardsTarget(int angle) {
    double angleDiff = angle - gyro.getAngle();
    double steerAmt = steerKp * angle;

    if (angleDiff > 1.0) {
      steerAmt += moveMin;
    } else if (angleDiff < 1.0) {
      steerAmt -= moveMin;
    }
    

    return -steerAmt;

  }


  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
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
