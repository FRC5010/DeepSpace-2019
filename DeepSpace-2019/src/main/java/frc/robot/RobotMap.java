/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.BallIntake;
import frc.robot.subsystems.BeakIntake;
import frc.robot.subsystems.DirectionSensor;
import frc.robot.subsystems.DistanceSensor;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shifter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.VisionAssistedDrive;
import frc.robot.subsystems.Wrist;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  public static WPI_TalonSRX rightMotor1;
  public static WPI_TalonSRX rightMotor2;
  public static WPI_TalonSRX elevatorMotor;
  public static WPI_TalonSRX elevatorMotor2;
  public static WPI_TalonSRX wristMotor;
  public static WPI_VictorSPX intakeMotor;
  public static DoubleSolenoid beakSolenoid;

  public static WPI_TalonSRX leftMotor1;
  public static WPI_TalonSRX leftMotor2;
  //public static TalonSRX leftMotor3;

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
  public static AHRS gyro;
  public static DirectionSensor direction;
  public static Elevator elevator;
  public static double moveMin = 0.2;
  public static Wrist wrist;
  public static BallIntake ballIntake;
  public static BeakIntake beakIntake;
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
  public static void initComp() {
    SmartDashboard.putString("Robot", "Competition");
    initRobotComponents();
    initMotorsComp();
    initSubsystems();
    initCommands();
  }

  public static void initPractice(){
    SmartDashboard.putString("Robot", "Practice");
   // RobotMap_Paths.init();
    initRobotComponents();
    initMotorsPrac();
    initSubsystems();
    initCommands();
  }

  public static void initRobotComponents() {
    builtInAccelerometer = new BuiltInAccelerometer(Accelerometer.Range.k4G);
    rightEncoder = new Encoder(0,1);
    leftEncoder = new Encoder(2,3);
    encoderPPR=480;
    gyro = new AHRS(Port.kUSB1);
    beakSolenoid = new DoubleSolenoid(2, 1);
    beakIntake = new BeakIntake();
    shiftSolenoid = new Solenoid(0);
    //wrist = new Wrist();
  }
  public static void initMotorsComp() {
    rightMotor1 = new WPI_TalonSRX(3);
    rightMotor2 = new WPI_TalonSRX(6);

    leftMotor1 = new WPI_TalonSRX(2);
    leftMotor2 = new WPI_TalonSRX(5);
    rightMotor1.setInverted(true);
    rightMotor2.setInverted(true);

    rightMotor2.set(com.ctre.phoenix.motorcontrol.ControlMode.Follower, 3);
		leftMotor2.set(com.ctre.phoenix.motorcontrol.ControlMode.Follower, 2);
    

    intakeMotor = new WPI_VictorSPX(0);
    
    wristMotor = new WPI_TalonSRX(4);

    elevatorMotor = new WPI_TalonSRX(7);
    elevatorMotor2 = new WPI_TalonSRX(1);
    elevatorMotor2.setInverted(true);
  }
  public static void initMotorsPrac() {
    rightMotor1 = new WPI_TalonSRX(4);
    rightMotor2 = new WPI_TalonSRX(6);

    leftMotor1 = new WPI_TalonSRX(2);
    leftMotor2 = new WPI_TalonSRX(5);
    rightMotor1.setInverted(true);
    rightMotor2.setInverted(true);

    rightMotor2.set(com.ctre.phoenix.motorcontrol.ControlMode.Follower, 4);
		leftMotor2.set(com.ctre.phoenix.motorcontrol.ControlMode.Follower, 2);
    

    intakeMotor = new WPI_VictorSPX(0);
    
    wristMotor = new WPI_TalonSRX(7);

    elevatorMotor = new WPI_TalonSRX(3);
    elevatorMotor2 = new WPI_TalonSRX(1);
  }

  public static void initSubsystems() {
    distance = new DistanceSensor();
    direction = new DirectionSensor(gyro);
    direction.reset();
    elevator = new Elevator(elevatorMotor, elevatorMotor2);
    shifter = new Shifter();
    driveTrain = new DriveTrain();
    vision = new Vision();
    vision.changePipeline(-1);
    visionDrive = new VisionAssistedDrive();
    ballIntake = new BallIntake();
    beakIntake = new BeakIntake();
    wrist = new Wrist();
  }

  public static void initCommands() {

  }

  public static void init() {
    //The init function for different robots.  Change based on functions above.
    File fieldMapFile = new File(Filesystem.getOperatingDirectory().toPath() + "/robot.txt");
    String data = "COMP";
    try {
      BufferedReader reader = new BufferedReader(new FileReader(fieldMapFile));
      while (reader.ready()) {
          data = reader.readLine();
      }
      if (data.compareToIgnoreCase("PRACTICE") == 0 && RobotBase.isReal()) {
        initPractice();
      } else if (data.compareToIgnoreCase("COMP") == 0  && RobotBase.isReal()) {
        initComp();
      } else if (data.compareToIgnoreCase("HOBBES") == 0 && RobotBase.isReal()) {
        RobotMapHobbes.initHobbes();
      } else if (data.compareToIgnoreCase("SIM") == 0 || RobotBase.isSimulation()) {
        RobotMapSim.initSim();
      } else {
        initComp();
      }
    } catch (FileNotFoundException e) {
      initComp();
    } catch (IOException e) {
      initComp();
    }
  }
  
}
