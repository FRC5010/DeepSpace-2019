/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.RaiseElevator;

/**
 * Add your docs here.
 */
public class Elevator extends Subsystem {

  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  TalonSRX elevMotor = RobotMap.elevatorMotor;
  Faults faults = new Faults();

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new RaiseElevator());
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  public void raiseElevator(double power) {
    elevMotor.set(ControlMode.PercentOutput, power);

  }

  public void moveToPosition(double setPoint) {
    RobotMap.elevatorMotor.set(ControlMode.MotionMagic,setPoint);
  }

  public double getCurrentPosition() {
    return RobotMap.elevatorMotor.getSelectedSensorPosition();
  }

  public void returnToManualControl() {
    RobotMap.elevatorMotor.set(ControlMode.PercentOutput, 0);
  }
  public void tuneElevator(double power){
    elevMotor.set(ControlMode.PercentOutput, power);
    elevMotor.getFaults(faults);
    System.out.println("Sensor Vel: " + elevMotor.getSelectedSensorVelocity());
    System.out.println("Sensor Pos: " + elevMotor.getSelectedSensorPosition());
    System.out.println("Elev Out %: " + elevMotor.getMotorOutputPercent());
    System.out.println("Out of Phs: " + faults.SensorOutOfPhase);
  }
}
