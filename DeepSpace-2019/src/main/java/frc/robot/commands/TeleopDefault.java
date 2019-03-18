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

  public TeleopDefault() {
    requires(RobotMap.driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
		SmartDashboard.putString("Command", this.getClass().getSimpleName());
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    moveAmt = Robot.oi.driveTrainForward.getValue();
    steerAmt = Robot.oi.driveTrainTurn.getValue();
    if (Robot.oi.speedReducerTrigger.getValue() != 0) {
      steerAmt *= 0.5;
      //System.out.println("speed reduced");
    } else {
      //System.out.println("speed normal");
    }
    RobotMap.driveTrain.drive(moveAmt + steerAmt, moveAmt - steerAmt);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
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
