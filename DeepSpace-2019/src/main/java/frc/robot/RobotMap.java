/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.subsystems.DriveTrain;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  public static TalonSRX rightMotor1;
  public static TalonSRX rightMotor2;
  public static TalonSRX rightMotor3;

  public static TalonSRX leftMotor1;
  public static TalonSRX leftMotor2;
  public static TalonSRX leftMotor3;

  public static DriveTrain driveTrain;

  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;

  public static void init() {
    rightMotor1 = new TalonSRX(4);
    rightMotor2 = new TalonSRX(5);
    rightMotor3 = new TalonSRX(6);

    leftMotor1 = new TalonSRX(1);
    leftMotor2 = new TalonSRX(2);
    leftMotor3 = new TalonSRX(3);

    rightMotor2.set(com.ctre.phoenix.motorcontrol.ControlMode.Follower, 4);
		rightMotor3.set(com.ctre.phoenix.motorcontrol.ControlMode.Follower, 4);

		leftMotor2.set(com.ctre.phoenix.motorcontrol.ControlMode.Follower, 1);
    leftMotor3.set(com.ctre.phoenix.motorcontrol.ControlMode.Follower, 1);

    driveTrain = new DriveTrain();
  }
}
