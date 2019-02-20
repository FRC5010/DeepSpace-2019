/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.RaiseElevator;
import frc.robot.util.Constants;
/**
 * Add your docs here.
 */
public class Elevator extends Subsystem {
  private double feedForward = 0.035;
  public static double ZERO = 1;
  public static double HATCH_LOW = 1;
  public static double HATCH_MIDDLE = 7263;
  public static double HATCH_HIGH = 14857;
  public static double CARGO_LOW = 6000;
  public static double CARGO_MIDDLE = 12900;
  public static double CARGO_HIGH = 19500;
  public static double CARGO_SHIP = 9100;
  public static enum Position { LOW, MIDDLE, HIGH }
  public boolean ballState = false;

  public Elevator(WPI_TalonSRX elevatorMotor, WPI_TalonSRX elevatorMotor2) {
    elevatorMotor.configFactoryDefault();
    elevatorMotor2.configFactoryDefault();
    elevatorMotor2.follow(elevatorMotor);
    elevatorMotor.setNeutralMode(NeutralMode.Brake);
    elevatorMotor2.setNeutralMode(NeutralMode.Brake);

    elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    elevatorMotor.setSensorPhase(true);
    elevatorMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
    elevatorMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);
    elevatorMotor.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);
    elevatorMotor2.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);
    //configing outputs
    elevatorMotor.configNominalOutputForward(0,Constants.kTimeoutMs);
    elevatorMotor.configNominalOutputReverse(0,Constants.kTimeoutMs);
    elevatorMotor.configPeakOutputForward(1,Constants.kTimeoutMs);
    elevatorMotor.configPeakOutputReverse(-1,Constants.kTimeoutMs);
    elevatorMotor.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx); 

    SmartDashboard.putNumber("Elevator P", Constants.kGains.kP);
    SmartDashboard.putNumber("Elevator I", Constants.kGains.kI);
    SmartDashboard.putNumber("Elevator D", Constants.kGains.kD);
    SmartDashboard.putNumber("Elevator F", Constants.kGains.kF);
    elevatorMotor.config_kF(Constants.kSlotIdx,Constants.kGains.kF, Constants.kTimeoutMs);  
    elevatorMotor.config_kP(Constants.kSlotIdx,Constants.kGains.kP, Constants.kTimeoutMs);
    elevatorMotor.config_kI(Constants.kSlotIdx,Constants.kGains.kI, Constants.kTimeoutMs);
    elevatorMotor.config_kD(Constants.kSlotIdx,Constants.kGains.kD, Constants.kTimeoutMs);
    elevatorMotor.configAllowableClosedloopError(Constants.kSlotIdx, 10, Constants.kTimeoutMs);
    elevatorMotor.config_IntegralZone(Constants.kSlotIdx, 100, Constants.kTimeoutMs);
    elevatorMotor.configClosedLoopPeakOutput(Constants.kSlotIdx, 1, Constants.kTimeoutMs);

    elevatorMotor.configClosedLoopPeriod(0, 1, Constants.kTimeoutMs);
    elevatorMotor.configClosedLoopPeriod(1, 1, Constants.kTimeoutMs);

    //cruise velocity
    elevatorMotor.configMotionCruiseVelocity(2500,Constants.kTimeoutMs);
    elevatorMotor.configMotionAcceleration(1500, Constants.kTimeoutMs);

    //zeroing sensor
    elevatorMotor.setSelectedSensorPosition(0,Constants.kPIDLoopIdx, Constants.kTimeoutMs);    
    SmartDashboard.putNumber("Elevator Feed Forward", feedForward);
  }
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  WPI_TalonSRX elevMotor = RobotMap.elevatorMotor;
  Faults faults = new Faults();

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new RaiseElevator());
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }



  public void raiseElevator(double power) {
    feedForward = SmartDashboard.getNumber("Elevator Feed Forward", 0.2);
    elevMotor.set(ControlMode.PercentOutput, power, DemandType.ArbitraryFeedForward, feedForward);
  }

  public void moveToPosition(double setPoint) {
    // double kP = SmartDashboard.getNumber("Elevator P", 4);
    // double kI = SmartDashboard.getNumber("Elevator I", 0);
    // double kD = SmartDashboard.getNumber("Elevator D", 4);
    // double kF = SmartDashboard.getNumber("Elevator F", 0.2);
    // elevMotor.config_kF(Constants.kSlotIdx,kF, Constants.kTimeoutMs);  
    // elevMotor.config_kP(Constants.kSlotIdx,kP, Constants.kTimeoutMs);
    // elevMotor.config_kI(Constants.kSlotIdx,kI, Constants.kTimeoutMs);
    // elevMotor.config_kD(Constants.kSlotIdx,kD, Constants.kTimeoutMs);
    RobotMap.elevatorMotor.set(ControlMode.MotionMagic, setPoint, DemandType.ArbitraryFeedForward, feedForward);
  }

  public double getCurrentPosition() {
    SmartDashboard.putNumber("Elevator position: ", elevMotor.getSelectedSensorPosition());
    return RobotMap.elevatorMotor.getSelectedSensorPosition();
  }

  public void returnToManualControl() {
    RobotMap.elevatorMotor.set(ControlMode.PercentOutput, 0);
  }

  public void tuneElevator(double power) {
    elevMotor.set(ControlMode.PercentOutput, power);
    elevMotor.getFaults(faults);
    System.out.println("Sensor Vel: " + elevMotor.getSelectedSensorVelocity());
    System.out.println("Sensor Pos: " + elevMotor.getSelectedSensorPosition());
    System.out.println("Elev Out %: " + elevMotor.getMotorOutputPercent());
    System.out.println("Out of Phs: " + faults.SensorOutOfPhase);
  }
}
