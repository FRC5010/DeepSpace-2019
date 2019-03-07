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
import frc.robot.subsystems.VisionAssistedDrive;

public class VisionAssistedSteering extends Command {
  VisionAssistedDrive vad;

  public VisionAssistedSteering () {
    requires(RobotMap.driveTrain);
    vad = RobotMap.visionDrive;
    vad.printPIDValues();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
		SmartDashboard.putString("Command", this.getClass().getSimpleName());
  }
 
  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double output = Robot.oi.driveTrainForward.getValue();
    //double turn = VisionAssistedDrive.arcTowardsTarget();
    double turn = vad.arcTowardsTarget();

    RobotMap.driveTrain.drive(output - turn, output + turn);
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
    end();
  }
}
