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

    switch (position) {
    case LOW: {
      setPoint = Wrist.CARGO_LOW;
      break;
    }
    case MIDDLE: {
      setPoint = Wrist.CARGO_MIDDLE;
      break;
    }
    case HIGH: {
      setPoint = Wrist.CARGO_HIGH;
      break;
    }
    }
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
		SmartDashboard.putString("Command", this.getClass().getSimpleName());
    SmartDashboard.putString("Wrist MM", "Initialized");
    SmartDashboard.putNumber("Wrist MM Setpoint", setPoint);
    if (RobotMap.wrist.lastMMPosition == Position.LOW) {
      RobotMap.wrist.reset();
    }
    RobotMap.wrist.lastMMPosition = this.position;
    if (RobotMap.elevator.isCargoGamePiece) {
       RobotMap.wrist.moveToPosition(setPoint);
    } else {
      RobotMap.wrist.moveToPosition(Wrist.CARGO_LOW);
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    SmartDashboard.putString("Wrist MM", "Running");
    if (RobotMap.elevator.isCargoGamePiece) {
      RobotMap.wrist.moveToPosition(setPoint);
    } else {
      RobotMap.wrist.moveToPosition(Wrist.CARGO_LOW);
    }

  }

  // Make this return true when this Command no longer needs to run execute()
  double timesAtPrevError;
  double prevError;

  @Override
  protected boolean isFinished() {
    double err = Math.abs(Math.abs(setPoint) -
      Math.abs(RobotMap.wrist.ticsToAngle(RobotMap.wrist.getCurrentPosition())));
    double lastError = err;
    double manualPower = Robot.oi.wristControl.getValue();
    // Counts how long the wrist is at the same sensor position
    if (((int) lastError) / 10 == ((int) prevError) / 10) {
      timesAtPrevError++;
    } else {
      timesAtPrevError = 0;
    }
    prevError = lastError;
    // if it gets stuck down make that sensor value 0 or if its stuck up make it go
    // down
    if (position == Position.LOW && timesAtPrevError > 20) {
      RobotMap.wristMotor.setSelectedSensorPosition(0);
    }
    SmartDashboard.putNumber("Wrist MM err", err);
    return manualPower != 0 // moving the joystick will abort MM
      || RobotMap.wrist.isSomethingStuck(RobotMap.wristMotor.getMotorOutputPercent())
      || err < 1
     || timesAtPrevError > 20; // This means we're close enough
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    if (position == Position.LOW) {
      RobotMap.wristMotor.setSelectedSensorPosition(0);
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
