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
import frc.robot.subsystems.Elevator;

public class RaiseElevator extends Command {
  private double moveUp;
  private double deadZone = 0.15;  

  public RaiseElevator() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(RobotMap.elevator);
  }

  private double scaleInputs(double input){
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
    moveUp = scaleInputs(Robot.oi.coDriver.getRawAxis(1));
    if(moveUp <= 0){
      moveUp = 0;
    }
    RobotMap.elevator.raiseElevator(moveUp);
    SmartDashboard.putNumber("Elevator power", moveUp);
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
