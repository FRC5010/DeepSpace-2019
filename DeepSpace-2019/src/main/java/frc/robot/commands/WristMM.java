/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.Robot;

public class WristMM extends Command {
  private double setPoint = 0;

  public WristMM(double setPoint) {
    this.setPoint = RobotMap.wrist.angleToTics(setPoint);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(RobotMap.wrist);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    SmartDashboard.putString("Wrist MM", "Initialized");
    SmartDashboard.putNumber("Wrist MM Setpoint", setPoint);

    RobotMap.wrist.moveToPosition(setPoint);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    SmartDashboard.putString("Wrist MM", "Running");
    RobotMap.wrist.moveToPosition(setPoint);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    double manualPower = Robot.oi.wristControl.getValue();
    double pos = RobotMap.wrist.getCurrentPosition();
    double err = Math.abs(setPoint - pos);
    SmartDashboard.putNumber("Wrist MM err", err);
    return manualPower != 0 || RobotMap.wrist.motorStuck() || err < 2000;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    SmartDashboard.putString("Wrist MM", "Done");
    RobotMap.wrist.returnToManualControl();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
