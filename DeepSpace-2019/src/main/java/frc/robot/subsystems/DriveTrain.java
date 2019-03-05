/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.TeleopDefault;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  TalonSRX rightMotors = RobotMap.rightMotor1;
  TalonSRX leftMotors = RobotMap.leftMotor1;

  @Override
  public void initDefaultCommand() {
    // Set a default command for a subsystem here.
    // setDefaultCommand(new TeleopDefault());
    setDefaultCommand(new TeleopDefault());
  }

  public void stop(){
    leftMotors.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, 0);
    rightMotors.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, 0);
  }
  
  public void drive(double leftPower, double rightPower){
    leftMotors.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, leftPower);
    rightMotors.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, rightPower);
  }
}


