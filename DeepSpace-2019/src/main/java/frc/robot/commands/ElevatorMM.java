/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.RobotMap;

public class ElevatorMM extends Command {
  double setPoint = 0;
  public ElevatorMM(double setPoint) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.setPoint = setPoint;
    requires(RobotMap.elevator);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    RobotMap.elevator.moveToPosition(setPoint);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    double pos = RobotMap.elevator.getCurrentPosition();
    double err = (Math.abs(setPoint-pos)/setPoint);
    
    return err < 0.01;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    RobotMap.elevator.returnToManualControl();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
