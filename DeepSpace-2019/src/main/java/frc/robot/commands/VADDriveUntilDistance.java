/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.command.PIDCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.subsystems.Pose;
import frc.robot.subsystems.Shifter;
import frc.robot.subsystems.VisionAssistedDrive;

public class VADDriveUntilDistance extends PIDCommand {

  public VADDriveUntilDistance () {
    super("VADDriveUntilDistance", 0, 0, 0);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    PIDController pid = getPIDController();
    pid.setP(Shifter.isLowGear ? VisionAssistedDrive.lowGear.moveKp : VisionAssistedDrive.highGear.moveKp);
    pid.setI(0);
    pid.setD(0);
    pid.setAbsoluteTolerance(1);
    pid.setInputRange(-30, 30);
    pid.setEnabled(true);
    pid.setSetpoint(0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    SmartDashboard.putString("VADDriveUntilDistance", "working");
    SmartDashboard.putNumber("VAD Error", getPIDController().getError());
  }

  @Override
  protected double returnPIDInput() {
    return Pose.getCurrentPose().limeLight.tX;
  }

  @Override
  protected void usePIDOutput(double output) {
    double driveOutput = VisionAssistedDrive.moveTowardsTarget();
    //double driveOutput = 0;
    output = 0;
    RobotMap.driveTrain.drive(driveOutput - output, driveOutput + output);
    SmartDashboard.putNumber("VADDriveUntilDistance Steer", output);
    SmartDashboard.putNumber("VADDriveUntilDistance Drive", driveOutput);
  }
  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Math.abs(Pose.getCurrentPose().limeLight.tY) < 1.0;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    SmartDashboard.putString("VADDriveUntilDistance", "end");
    RobotMap.driveTrain.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
