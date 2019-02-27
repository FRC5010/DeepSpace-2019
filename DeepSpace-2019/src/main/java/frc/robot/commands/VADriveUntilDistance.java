/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.sql.Struct;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.DirectionSensor;
import frc.robot.subsystems.Pose;
import frc.robot.subsystems.Shifter;
import frc.robot.subsystems.VisionAssistedDrive;

public class VADriveUntilDistance extends Command {
  private double last_heading_error = 0;
  private double lastError = 0;
  private double setpoint = 0;

  public VADriveUntilDistance () {
    SmartDashboard.putNumber("moveKp", getMoveKp());
    SmartDashboard.putNumber("moveKd", getMoveKd());
    SmartDashboard.putNumber("steerKp", getSteerKp());
    SmartDashboard.putNumber("steerKd", getSteerKd());
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
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

    double turn = turnTowards();

    RobotMap.driveTrain.drive(output - turn, output + turn);
    //SmartDashboard.putNumber("VADDriveUntilDistance Drive", output);
    System.out.println("VADDriveUntilDistance: " + output);
  }
  public double moveTowardsTarget(double error, double lastError) {
    double moveAmt = 0;
    if (Pose.getCurrentPose().limeLight.tValid) {
      double tY = Pose.getCurrentPose().limeLight.tY;
      double error_delta = error - lastError;
      moveAmt = getMoveKp() * tY +  getMoveKd() * error_delta;

      double moveMin = Shifter.isLowGear ? VisionAssistedDrive.lowGear.moveMin : VisionAssistedDrive.highGear.moveMin;
      moveAmt = Math.max(moveMin, Math.abs(moveAmt)) * Math.signum(moveAmt);
    }
    return moveAmt;
  }

  private double getMoveKp() {
    double kP = (Shifter.isLowGear ? VisionAssistedDrive.lowGear.moveKp : VisionAssistedDrive.highGear.moveKp);
    //kP = SmartDashboard.getNumber("moveKp", kP);
    return kP;
  }
  private double getMoveKd() {
    double kD = (Shifter.isLowGear ? VisionAssistedDrive.lowGear.moveKd : VisionAssistedDrive.highGear.moveKd);
    //kD = SmartDashboard.getNumber("moveKd", kD);
    return kD;
  }
  private double getSteerKp() {
    double kP = (Shifter.isLowGear ? VisionAssistedDrive.lowGear.steerKp : VisionAssistedDrive.highGear.steerKp);
    //kP = SmartDashboard.getNumber("steerKp", kP);
    return kP;
  }
  private double getSteerKd() {
    double kD = (Shifter.isLowGear ? VisionAssistedDrive.lowGear.steerKd : VisionAssistedDrive.highGear.steerKd);
    //kD = SmartDashboard.getNumber("steerKd", kD);
    return kD;
  }

  double turnTowards() {
    double heading = Pose.getCurrentPose().limeLight.tX;
    double heading_error = DirectionSensor.boundHalfDegrees(0 - heading);
    double heading_delta = heading_error - last_heading_error;
    last_heading_error = heading_error;

    double steerKp = getSteerKp();
    double steerKd = getSteerKd();

    double turnAmt = steerKp * heading_error + (steerKd * heading_delta);
    double moveMin = Shifter.isLowGear ? VisionAssistedDrive.lowGear.moveMin : VisionAssistedDrive.highGear.moveMin;
    turnAmt = Math.max(moveMin, Math.abs(turnAmt)) * Math.signum(turnAmt);
    SmartDashboard.putNumber("VADDriveUntilDistance Steer", turnAmt);
    return turnAmt;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    double manualOverride = Robot.oi.driveTrainForward.getValue();
    double steerOverride = Robot.oi.driveTrainTurn.getValue();
    return 0 != manualOverride || 0 != steerOverride || 
    Math.abs(lastError) < .6;
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
  }
}
