/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.sql.Struct;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.DirectionSensor;
import frc.robot.subsystems.Pose;
import frc.robot.subsystems.Shifter;
import frc.robot.subsystems.VisionAssistedDrive;

public class VADriveUntilDistance extends Command {
  VisionAssistedDrive vad;

  private double lastHeadingError = 0;
  private double lastError = 0;
  private double setpoint = 0;
  private double prevError = 0;
  private int timesAtPrevError = 0;

  public VADriveUntilDistance (double setpoint) {
    this.setpoint = setpoint;
    vad = RobotMap.visionDrive;
    vad.printPIDValues();
    requires(RobotMap.driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
		SmartDashboard.putString("Command", this.getClass().getSimpleName());
    SmartDashboard.putString("VADDriveUntilDistance", "init");
    lastError = 0;
    lastHeadingError = 0;
    prevError = 0;
    timesAtPrevError = 0;
    moveTowardsTarget();
  }
 
  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    SmartDashboard.putString("VADDriveUntilDistance", "working");

    double output = moveTowardsTarget();
    // output = 0;
    double turn = VisionAssistedDrive.arcTowardsTarget();
    //    double turn = turnTowards();

    RobotMap.driveTrain.drive(output - turn, output + turn);
  }

  double moveTowardsTarget() {
    double moveAmt = 0;
    if (Pose.getCurrentPose().limeLight.tValid) {
      double distance = Pose.getCurrentPose().limeLight.tDistance;
      double error =  distance - setpoint;
      double errorDelta = error - lastError;

      SmartDashboard.putNumber("VADDriveUntilDistance distance", distance);
      moveAmt = vad.getMoveKp() * error + vad.getMoveKd() * errorDelta;

      double moveMin = vad.getMoveMin();
      moveAmt = Math.max(moveMin, Math.abs(moveAmt)) * Math.signum(moveAmt);
      lastError = error;
    }
    return moveAmt;
  }

  double turnTowards() {
    double turnAmt = 0;
    if (Pose.getCurrentPose().limeLight.tValid) {
      double heading = Pose.getCurrentPose().limeLight.tX;
      double headingError = DirectionSensor.boundHalfDegrees(0 - heading);
      double headingDelta = headingError - lastHeadingError;

      turnAmt = vad.getSteerKp() * headingError + vad.getSteerKd() * headingDelta;

      double steerMin = vad.getSteerMin();
      turnAmt = Math.max(steerMin, Math.abs(turnAmt)) * Math.signum(turnAmt);
      lastHeadingError = headingError;
    }
    SmartDashboard.putNumber("VADDriveUntilDistance Steer", turnAmt);
    return turnAmt;
  }

  // Make this return true when this Command no longer needs to run execute()

  @Override
  protected boolean isFinished() {
    double manualOverride = Robot.oi.driveTrainForward.getValue();
    double steerOverride = Robot.oi.driveTrainTurn.getValue();
    SmartDashboard.putNumber("manualOverride", manualOverride);
    SmartDashboard.putNumber("steerOverride", steerOverride);
    SmartDashboard.putNumber("lastError", lastError);
    SmartDashboard.putNumber("timesAtPrevError", timesAtPrevError);
    SmartDashboard.putNumber("prevError", prevError);
    if ( ((int)lastError)/1 == ((int)prevError)/1 ) {
      timesAtPrevError++;
    } else {
      timesAtPrevError = 0;
    }
    prevError = lastError;
    return 0 != manualOverride || 0 != steerOverride || 
      Math.abs(lastError) < 1 || timesAtPrevError > 50;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    SmartDashboard.putString("VADDriveUntilDistance", "end");
    RobotMap.driveTrain.stop();
    lastError = 0;
    lastHeadingError = 0;
    prevError = 0;
    timesAtPrevError = 0;
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
