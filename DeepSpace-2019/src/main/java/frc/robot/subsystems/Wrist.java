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
import frc.robot.commands.MoveWrist;
import frc.robot.util.Constants;
/**
 * Add your docs here.
 */
public class Wrist extends Subsystem {
  public static double lowestAngle = 32;
  public static double feedForward = 0.195;
  public static double ZERO = 1;
  public static double HATCH_LOW = -lowestAngle + 5;
  public static double HATCH_MIDDLE = 0;
  public static double HATCH_HIGH = 70;
  public static double CARGO_LOW = -lowestAngle + 5;
  public static double CARGO_MIDDLE = 0;
  public static double CARGO_HIGH = 70;
  public static double CARGO_SHIP = 0;

  public Wrist() {
    wristMotor.configFactoryDefault();
    wristMotor.setNeutralMode(NeutralMode.Brake);
    wristMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    wristMotor.setSensorPhase(false);
    wristMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
    wristMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);
    wristMotor.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);
    //configing outputs
    wristMotor.configNominalOutputForward(0,Constants.kTimeoutMs);
    wristMotor.configNominalOutputReverse(0,Constants.kTimeoutMs);
    wristMotor.configPeakOutputForward(.5,Constants.kTimeoutMs);
    wristMotor.configPeakOutputReverse(-.3,Constants.kTimeoutMs);
    wristMotor.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx); 
    
    SmartDashboard.putNumber("Wrist P", Constants.wristGains.kP);
    SmartDashboard.putNumber("Wrist I", Constants.wristGains.kI);
    SmartDashboard.putNumber("Wrist D", Constants.wristGains.kD);
    SmartDashboard.putNumber("Wrist F", Constants.wristGains.kF);
    
    wristMotor.config_kF(Constants.kSlotIdx,Constants.wristGains.kF, Constants.kTimeoutMs);  
    wristMotor.config_kP(Constants.kSlotIdx,Constants.wristGains.kP, Constants.kTimeoutMs);
    wristMotor.config_kI(Constants.kSlotIdx,Constants.wristGains.kI, Constants.kTimeoutMs);
    wristMotor.config_kD(Constants.kSlotIdx,Constants.wristGains.kD, Constants.kTimeoutMs);
    wristMotor.configAllowableClosedloopError(Constants.kSlotIdx, 5000, Constants.kTimeoutMs);
    wristMotor.config_IntegralZone(Constants.kSlotIdx, 100, Constants.kTimeoutMs);
    
    wristMotor.configClosedLoopPeakOutput(0, .5, Constants.kTimeoutMs);

    wristMotor.configClosedLoopPeriod(0, 1, Constants.kTimeoutMs);
    wristMotor.configClosedLoopPeriod(1, 1, Constants.kTimeoutMs);
  
    //cruise velocity
    wristMotor.configMotionCruiseVelocity(9000,Constants.kTimeoutMs);
    wristMotor.configMotionAcceleration(9000, Constants.kTimeoutMs);

    //zeroing sensor
    wristMotor.setSelectedSensorPosition(0,Constants.kSlotIdx, Constants.kTimeoutMs);
    SmartDashboard.putNumber("Wrist Feed Forward", feedForward);
  }
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
    feedForward = SmartDashboard.getNumber("Wrist Feed Forward", 0.195);
    if (Math.signum(power) == -1) {
      power = Math.max(-.15, power);
    } else {
      power = Math.min(.35, power);
    }
    SmartDashboard.putNumber("Wrist power", power);

    wristMotor.set(ControlMode.PercentOutput, power, DemandType.ArbitraryFeedForward, feedForward);
  }

  public void moveToPosition(double setPoint) {
    double kP = SmartDashboard.getNumber("Wrist P", 4);
    double kI = SmartDashboard.getNumber("Wrist I", 0);
    double kD = SmartDashboard.getNumber("Wrist D", 4);
    double kF = SmartDashboard.getNumber("Wrist F", 0.2);
    feedForward = SmartDashboard.getNumber("Wrist Feed Forward", 0.2);
    wristMotor.config_kF(Constants.kSlotIdx,kF, Constants.kTimeoutMs);  
    wristMotor.config_kP(Constants.kSlotIdx,kP, Constants.kTimeoutMs);
    wristMotor.config_kI(Constants.kSlotIdx,kI, Constants.kTimeoutMs);
    wristMotor.config_kD(Constants.kSlotIdx,kD, Constants.kTimeoutMs);
    wristMotor.set(ControlMode.Position, setPoint, DemandType.ArbitraryFeedForward, feedForward);
    SmartDashboard.putNumber("Wrist Vel", wristMotor.getActiveTrajectoryVelocity());
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

  public double angleToTics(double angle) {
    return (angle + lowestAngle) * 1100;
  }

  public double ticsToAngle(double tics) {
    return (tics / 1100) - lowestAngle;
  }
}
