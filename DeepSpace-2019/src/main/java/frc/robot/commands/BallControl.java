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
import frc.robot.subsystems.Wrist;

public class BallControl extends Command {
  private double move;
  private double leftTrigger;
  private double rightTrigger;

  public BallControl() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(RobotMap.ballIntake);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
		SmartDashboard.putString("Command", this.getClass().getSimpleName());
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    leftTrigger = Robot.oi.ballIntake.getValue();
    rightTrigger = Robot.oi.ballOuttake.getValue();
    SmartDashboard.putNumber("leftTrigger: ", leftTrigger);
    SmartDashboard.putNumber("rightTrigger: ", rightTrigger);
    if (leftTrigger != 0) {
      move = leftTrigger;
    } else if (rightTrigger != 0) {
      move = rightTrigger;
    } else if (RobotMap.wristMotor.getSelectedSensorPosition() < 100000) {
      //move = 0-.1;
    }

    RobotMap.ballIntake.ballControl(move);
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
