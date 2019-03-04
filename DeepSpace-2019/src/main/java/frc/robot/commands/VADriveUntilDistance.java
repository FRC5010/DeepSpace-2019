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

  public VADriveUntilDistance (double setpoint) {
    this.setpoint = setpoint;
    vad = RobotMap.visionDrive;
    vad.printPIDValues();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
		SmartDashboard.putString("Command", VADriveUntilDistance.class.getName());
    SmartDashboard.putString("VADDriveUntilDistance", "init");
  }
 
  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    SmartDashboard.putString("VADDriveUntilDistance", "working");
    double error = setpoint - Pose.getCurrentPose().limeLight.tY;
    double output = moveTowardsTarget(error, lastError);
    lastError = error;
    // Uncomment this to tune steering
   // output = 0;

    double turn = VisionAssistedDrive.arcTowardsTarget(); //turnTowards();

    RobotMap.driveTrain.drive(output - turn, output + turn);
    //SmartDashboard.putNumber("VADDriveUntilDistance Drive", output);
  }
  public double moveTowardsTarget(double error, double lastError) {
    double moveAmt = 0;
    if (Pose.getCurrentPose().limeLight.tValid) {
      double tY = Pose.getCurrentPose().limeLight.tDistance;
      double error_delta = error - lastError;
      moveAmt = vad.getMoveKp() * tY +  vad.getMoveKd() * error_delta;

      double moveMin = vad.getMoveMin();
      moveAmt = Math.max(moveMin, Math.abs(moveAmt)) * Math.signum(moveAmt);
    }
    return moveAmt;
  }

  double turnTowards() {
    double heading = Pose.getCurrentPose().limeLight.tX;
    double heading_error = DirectionSensor.boundHalfDegrees(0 - heading);
    double heading_delta = heading_error - lastHeadingError;
    lastHeadingError = heading_error;

    double steerKp = vad.getSteerKp();
    double steerKd = vad.getSteerKd();

    double turnAmt = steerKp * heading_error + (steerKd * heading_delta);
    double steerMin = vad.getSteerMin();
    turnAmt = Math.max(steerMin, Math.abs(turnAmt)) * Math.signum(turnAmt);
    SmartDashboard.putNumber("VADDriveUntilDistance Steer", turnAmt);
    return turnAmt;
  }

  // Make this return true when this Command no longer needs to run execute()
  private static double prevError;
  private static int timesAtPrevError;
  @Override
  protected boolean isFinished() {
    double manualOverride = Robot.oi.driveTrainForward.getValue();
    double steerOverride = Robot.oi.driveTrainTurn.getValue();
    if ( ((int)lastError)/1 == ((int)prevError)/1 ) {
      timesAtPrevError++;
    } else {
      timesAtPrevError = 0;
    }
    prevError = lastError;
    return 0 != manualOverride || 0 != steerOverride || 
      Math.abs(lastError) < .6 || timesAtPrevError > 50;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    SmartDashboard.putString("VADDriveUntilDistance", "end");
    RobotMap.driveTrain.stop();
    lastError = 0;
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
