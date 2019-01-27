/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.subsystems.Shifter;
import frc.robot.subsystems.DirectionSensor;
import frc.robot.subsystems.DistanceSensor;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.VisionAssistedDrive;

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

  public static Solenoid shiftSolenoid;
  
  public static Accelerometer builtInAccelerometer;

  public static Shifter shifter;
  public static DriveTrain driveTrain;
  public static Vision vision;
  public static VisionAssistedDrive visionDrive;
  public static Encoder leftEncoder;
  public static Encoder rightEncoder;
  public static int encoderPPR;
  public static DistanceSensor distance;
  public static Gyro gyro;
  public static DirectionSensor direction;
  public static Elevator elevator;
  public static double moveMin = 0.2;
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;

  public static void init() {
    RobotMap_Paths.init();
    rightMotor1 = new TalonSRX(4);
    rightMotor2 = new TalonSRX(5);
    rightMotor3 = new TalonSRX(6);

    leftMotor1 = new TalonSRX(1);
    leftMotor2 = new TalonSRX(2);
    leftMotor3 = new TalonSRX(3);

    //Inverted for motion profiling purposes.
    /*leftMotor1.setInverted(true);
    leftMotor2.setInverted(true);
    leftMotor3.setInverted(true);*/

    rightMotor1.setInverted(true);
    rightMotor2.setInverted(true);
    rightMotor3.setInverted(true);

    rightMotor2.set(com.ctre.phoenix.motorcontrol.ControlMode.Follower, 4);
		rightMotor3.set(com.ctre.phoenix.motorcontrol.ControlMode.Follower, 4);

		leftMotor2.set(com.ctre.phoenix.motorcontrol.ControlMode.Follower, 1);
    leftMotor3.set(com.ctre.phoenix.motorcontrol.ControlMode.Follower, 1);
    
    shiftSolenoid = new Solenoid(1);

    builtInAccelerometer = new BuiltInAccelerometer(Accelerometer.Range.k4G);
    rightEncoder = new Encoder(0,1);
    leftEncoder = new Encoder(2,3);
    encoderPPR=480;
    distance = new DistanceSensor();
    gyro = new ADXRS450_Gyro(); 
    direction = new DirectionSensor();
    direction.reset();

    elevator = new Elevator();
      shifter = new Shifter();
    driveTrain = new DriveTrain();
    vision = new Vision();
    visionDrive = new VisionAssistedDrive();
  }
}
