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
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.MoveWrist;

/**
 * Add your docs here.
 */
public class Wrist extends Subsystem {

  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  WPI_TalonSRX wristMotor = RobotMap.wristMotor;
  Faults faults = new Faults();

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new MoveWrist());
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void moveWrist(double power) {
    wristMotor.set(ControlMode.PercentOutput, power);

  }

  public void moveToPosition(double setPoint) {
    wristMotor.set(ControlMode.MotionMagic, setPoint);
  }

  public double getCurrentPosition() {
    return wristMotor.getSelectedSensorPosition();
  }

  public void returnToManualControl() {
    wristMotor.set(ControlMode.PercentOutput, 0);
  }

  public void tuneWrist(double power) {
    wristMotor.set(ControlMode.PercentOutput, power);
    wristMotor.getFaults(faults);
    // System.out.println("Sensor Vel: " + wristMotor.getSelectedSensorVelocity());
    // System.out.println("Sensor Pos: " + wristMotor.getSelectedSensorPosition());
    System.out.println("wrist Out %: " + wristMotor.getMotorOutputPercent());
    // System.out.println("Out of Phs: " + faults.SensorOutOfPhase);
  }
}
