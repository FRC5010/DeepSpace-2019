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
import frc.robot.commands.groups.Preload;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.Position;
import frc.robot.Robot;

public class WristMM extends Command {
  private double setPoint = 0;
  private Wrist.Position position;

  public WristMM(Wrist.Position Position) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.position = Position;

    //System.out.println("Wrist Position: "+ position);
    requires(RobotMap.wrist);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
		SmartDashboard.putString("Command", this.getClass().getSimpleName());
    SmartDashboard.putString("Wrist MM", "Initialized");
    setPoint = Wrist.HATCH_LOW;
    switch (position) {
      case LOW: {
        if (RobotMap.elevator.isCargoGamePiece || Preload.isPreloading) {
          setPoint = Wrist.CARGO_LOW;
        } 
        break;
      }
      case MIDDLE: {
        if (RobotMap.elevator.isCargoGamePiece || Preload.isPreloading) {
          setPoint = Wrist.CARGO_MIDDLE;
        }
        break;
      }
      case HIGH: {
        if (RobotMap.elevator.isCargoGamePiece || Preload.isPreloading) {
          setPoint = Wrist.CARGO_HIGH;
        }
        break;
      }
      case PRELOAD: {
        if (RobotMap.elevator.isCargoGamePiece) {
          setPoint = Wrist.CARGO_SHIP;
        } else {
          setPoint = Wrist.PRELOAD;
        }
       }
      }
      SmartDashboard.putNumber("Wrist MM Setpoint", setPoint);
    if (Wrist.lastMMPosition == Position.LOW) {
      RobotMap.wrist.reset();
    }
    RobotMap.wrist.lastMMPosition = this.position;
    SmartDashboard.putString("Wrist MM Position", position.toString());
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    SmartDashboard.putString("Wrist MM", "Running");
    RobotMap.wrist.moveToPosition(setPoint);
  }

  // Make this return true when this Command no longer needs to run execute()
  double timesAtPrevError;

  @Override
  protected boolean isFinished() {
    double err = Math.abs(setPoint -
      RobotMap.wrist.ticsToAngle(RobotMap.wrist.getCurrentPosition()));
    double lastError = err;
    double manualPower = Robot.oi.wristControl.getValue();
    // if it gets stuck down make that sensor value 0 or if its stuck up make it go
    // down
    if (err < 5) {
      RobotMap.wristMotor.setSelectedSensorPosition((int)RobotMap.wrist.angleToTics(setPoint));
    }
    return manualPower != 0 // moving the joystick will abort MM
      || RobotMap.wrist.isSomethingStuck(RobotMap.wristMotor.getMotorOutputPercent())
      || err < 5;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    if (position == Position.LOW) {
      RobotMap.wristMotor.setSelectedSensorPosition(RobotMap.wrist.angleToTics(Wrist.CARGO_LOW));
    }
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
