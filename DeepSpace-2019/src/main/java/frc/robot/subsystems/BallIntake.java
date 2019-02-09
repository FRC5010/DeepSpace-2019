/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.BallControl;

/**
 * Add your docs here.
 */
public class BallIntake extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  VictorSPX intakeMotor = RobotMap.intakeMotor;

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new BallControl());
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void ballControl(double power) {
    intakeMotor.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, power);
  }
  public void suck(double power){
    intakeMotor.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, -power);
  }
}
