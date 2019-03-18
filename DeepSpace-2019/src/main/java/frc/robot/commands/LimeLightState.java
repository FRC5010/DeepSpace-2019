/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.Vision;

public class LimeLightState extends Command {
  public static enum State { AUTO, DRIVER, BLINK_ON, BLINK_OFF }
  State state;
  long endTime = 0;
  boolean done = true;

  public LimeLightState(State state) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.state = state;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    switch (state) {
      case AUTO: {
        RobotMap.vision.setLimeLightLEDMode(Vision.LEDMode.ON);
        RobotMap.vision.setCamMode(Vision.CamMode.VISION);
        done = true;
        break;
      }      
      case DRIVER: {
        RobotMap.vision.setLimeLightLEDMode(Vision.LEDMode.BLINK);
        RobotMap.vision.setCamMode(Vision.CamMode.DRIVER);
        endTime = RobotController.getFPGATime() + 500000;
        done = false;
        break;
      }
      case BLINK_ON: {
        RobotMap.vision.setLimeLightLEDMode(Vision.LEDMode.BLINK);
        endTime = RobotController.getFPGATime() + 500000;
        done = false;
        break;
      }
      case BLINK_OFF: {
        RobotMap.vision.setLimeLightLEDMode(Vision.LEDMode.OFF);
        done = true;
        break;
      }
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (endTime > RobotController.getFPGATime()) {
      done = true;
      RobotMap.vision.setLimeLightLEDMode(Vision.LEDMode.OFF);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return done;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    done = true;
  }
}
