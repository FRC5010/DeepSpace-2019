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
  private Wrist.Position position = Wrist.Position.LOW;
  public WristMM( Wrist.Position Position) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.position = Position;
    requires(RobotMap.wrist);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
   
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
    


    SmartDashboard.putString("Wrist MM", "Initialized");
    SmartDashboard.putNumber("Wrist MM Setpoint", setPoint);

    RobotMap.wrist.moveToPosition(setPoint);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    SmartDashboard.putString("Wrist MM", "Running");
    RobotMap.wrist.moveToPosition(setPoint);
  }

  // Make this return true when this Command no longer needs to run execute()
  double timesAtPrevError;
  double prevError;
  @Override
  protected boolean isFinished() {
    double err = Math.abs(setPoint - RobotMap.wrist.getCurrentPosition());
    double lastError = err;
    double manualPower = Robot.oi.wristControl.getValue();

    //Counts how long the wrist is at the same sensor position
    if ( ((int)lastError)/1 == ((int)prevError)/1 ) {
      timesAtPrevError++;
    } else {
      timesAtPrevError = 0;
    }
    prevError=lastError;
//if it gets stuck down make that sensor value 0 or if its stuck up make it go down
    if(position==Position.LOW &&timesAtPrevError>50){
      RobotMap.wristMotor.setSelectedSensorPosition(0);
    }else if(position==Position.HIGH &&timesAtPrevError>50){
      RobotMap.wrist.moveToPosition(Wrist.CARGO_LOW);
    }
    SmartDashboard.putNumber("Wrist MM err", err);
    return manualPower != 0 // moving the joystick will abort MM
      || RobotMap.wrist.isSomethingStuck(RobotMap.wristMotor.getMotorOutputPercent())
      || err < 2000
      || timesAtPrevError>50; // This means we're close enough
  }
   
  // Called once after isFinished returns true
  @Override
  protected void end() {
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
