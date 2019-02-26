/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.PIDCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.DirectionSensor;
import frc.robot.subsystems.Pose;
import frc.robot.subsystems.Shifter;
import frc.robot.subsystems.VisionAssistedDrive;

public class VADriveUntilDistance extends PIDCommand {
  private double last_heading_error = 0;

  public VADriveUntilDistance () {
    super("VADDriveUntilDistance", 0, 0, 0);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    PIDController pid = getPIDController();
    pid.setP(Shifter.isLowGear ? VisionAssistedDrive.lowGear.moveKp : VisionAssistedDrive.highGear.moveKp);
    pid.setI(0);
    pid.setD(0);
    pid.setAbsoluteTolerance(0.5);
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
    return Pose.getCurrentPose().limeLight.tY;
  }

  @Override
  protected void usePIDOutput(double output) {
//    double driveOutput = VisionAssistedDrive.moveTowardsTarget();
    double moveMin = Shifter.isLowGear ? VisionAssistedDrive.lowGear.moveMin : VisionAssistedDrive.highGear.moveMin;
    output = Math.max(moveMin, Math.abs(output)) * Math.signum(output);
    // Uncomment this to tune steering
    // output = 0;

    double heading = Pose.getCurrentPose().limeLight.tX;
    double desired_heading = 0;
    double heading_error = DirectionSensor.boundHalfDegrees(desired_heading - heading);
    double heading_delta = heading_error - last_heading_error;
    // Use this to tune the D factor
    double steerKd = 0;

    double turn = heading_error * (Shifter.isLowGear ? VisionAssistedDrive.lowGear.steerKp : VisionAssistedDrive.highGear.steerKp)
        + (steerKd * heading_delta);

    RobotMap.driveTrain.drive(output + turn, output - turn);
    last_heading_error = heading_error;
    SmartDashboard.putNumber("VADDriveUntilDistance Steer", turn);
    SmartDashboard.putNumber("VADDriveUntilDistance Drive", output);
  }
  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    double manualOverride = Robot.oi.driveTrainForward.getValue();
    return 0 != manualOverride || getPIDController().onTarget();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    SmartDashboard.putString("VADDriveUntilDistance", "end");
    RobotMap.driveTrain.stop();
    getPIDController().reset();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
