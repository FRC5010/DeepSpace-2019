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
    SmartDashboard.putString(this.getClass().getSimpleName(), "init");
    lastError = 0;
    lastHeadingError = 0;
    prevError = 0;
    timesAtPrevError = 0;
  }
 
  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    SmartDashboard.putString(this.getClass().getSimpleName(), "working");

    double output = vad.moveTowardsTarget(setpoint, lastError);
    //double turn = VisionAssistedDrive.arcTowardsTarget();
    double turn = vad.turnTowards(0, lastHeadingError);

    RobotMap.driveTrain.drive(output - turn, output + turn);

    SmartDashboard.putNumber("Move error", lastError);
    SmartDashboard.putNumber("Steer error", lastHeadingError);
  }

  // Make this return true when this Command no longer needs to run execute()

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
    return 0 != manualOverride || 0 != steerOverride ||  (Math.abs(lastHeadingError) < 1 && Math.abs(lastError) < 1); // || timesAtPrevError > 50;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    SmartDashboard.putString(this.getClass().getSimpleName(), "end");
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
