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
import frc.robot.commands.WristMM;
import frc.robot.util.Constants;

/**
 * Add your docs here.
 */
public class Wrist extends Subsystem {
  public static double lowestAngle = 32;
  public static final double feedForward = 0.195;
  public static double ZERO = 1;
  public static double HATCH_LOW = -lowestAngle + 5;
  public static double HATCH_MIDDLE = 0;
  public static double HATCH_HIGH = 0;
  public static double CARGO_LOW = -lowestAngle + 5;
  public static double CARGO_MIDDLE = 0;
  public static double CARGO_HIGH = 70;
  public static double CARGO_SHIP = 0;
  public static double MAX_FWD_OUT = 1;
  public static double MAX_REV_OUT = -0.3;
  private long lastPosition = 0;
  private int numTimesAtLastPosition = 0;
  public static enum Position {
    LOW, MIDDLE, HIGH
  }

  public Wrist() {
    wristMotor.configFactoryDefault();
    wristMotor.setNeutralMode(NeutralMode.Brake);
    wristMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    wristMotor.setSensorPhase(false);
    wristMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
    wristMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);
    wristMotor.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);
    // configing outputs
    wristMotor.configNominalOutputForward(0, Constants.kTimeoutMs);
    wristMotor.configNominalOutputReverse(0, Constants.kTimeoutMs);
    wristMotor.configPeakOutputForward(MAX_FWD_OUT, Constants.kTimeoutMs);
    wristMotor.configPeakOutputReverse(MAX_REV_OUT, Constants.kTimeoutMs);

    SmartDashboard.putNumber("Wrist P", Constants.wristGains.kP);
    SmartDashboard.putNumber("Wrist I", Constants.wristGains.kI);
    SmartDashboard.putNumber("Wrist D", Constants.wristGains.kD);
    SmartDashboard.putNumber("Wrist F", Constants.wristGains.kF);

    wristMotor.config_kF(Constants.kSlotIdx, Constants.wristGains.kF, Constants.kTimeoutMs);
    wristMotor.config_kP(Constants.kSlotIdx, Constants.wristGains.kP, Constants.kTimeoutMs);
    wristMotor.config_kI(Constants.kSlotIdx, Constants.wristGains.kI, Constants.kTimeoutMs);
    wristMotor.config_kD(Constants.kSlotIdx, Constants.wristGains.kD, Constants.kTimeoutMs);

    wristMotor.config_kF(Constants.kSlotDown, Constants.wristDownGains.kF, Constants.kTimeoutMs);
    wristMotor.config_kP(Constants.kSlotDown, Constants.wristDownGains.kP, Constants.kTimeoutMs);
    wristMotor.config_kI(Constants.kSlotDown, Constants.wristDownGains.kI, Constants.kTimeoutMs);
    wristMotor.config_kD(Constants.kSlotDown, Constants.wristDownGains.kD, Constants.kTimeoutMs);

    wristMotor.configAllowableClosedloopError(Constants.kSlotIdx, 5000, Constants.kTimeoutMs);
    wristMotor.config_IntegralZone(Constants.kSlotIdx, 100, Constants.kTimeoutMs);

    wristMotor.configAllowableClosedloopError(Constants.kSlotDown, 5000, Constants.kTimeoutMs);
    wristMotor.config_IntegralZone(Constants.kSlotDown, 100, Constants.kTimeoutMs);

    wristMotor.configClosedLoopPeakOutput(Constants.kSlotIdx, MAX_FWD_OUT, Constants.kTimeoutMs);
    wristMotor.configClosedLoopPeakOutput(Constants.kSlotDown, Math.abs(MAX_REV_OUT), Constants.kTimeoutMs);

    wristMotor.configClosedLoopPeriod(0, 1, Constants.kTimeoutMs);
    wristMotor.configClosedLoopPeriod(1, 1, Constants.kTimeoutMs);

    // cruise velocity
    wristMotor.configMotionCruiseVelocity(9000, Constants.kTimeoutMs);
    wristMotor.configMotionAcceleration(9000, Constants.kTimeoutMs);

    // zeroing sensor
    wristMotor.setSelectedSensorPosition(0, Constants.kSlotIdx, Constants.kTimeoutMs);
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
  public void reset(){
    RobotMap.wristMotor.setSelectedSensorPosition(0);
  }
  private double calculateFeedForward() {
    long tics = getCurrentPosition();
    SmartDashboard.putNumber("Wrist position", tics);
    double angle = RobotMap.wrist.ticsToAngle(tics);
    SmartDashboard.putNumber("Wrist Angle", angle);
    double cosine = Math.cos(Math.toRadians(angle));
    double feedForward = SmartDashboard.getNumber("Wrist Feed Forward", Wrist.feedForward);
    double calcFF = feedForward * cosine;
    SmartDashboard.putNumber("Wrist Calc FF", calcFF);
    return calcFF;
  }

  /** Don't drive the motor past a point where it can't move for too long */
  public boolean isSomethingStuck(double power) {
    if (RobotMap.checkMotorSafety && 0.25 < power) {
      long curPose = getCurrentPosition();
      if (lastPosition / 100 == curPose / 100) {
        ++numTimesAtLastPosition;
      } else {
        numTimesAtLastPosition = 0;
      }
      lastPosition = curPose;
      return 3 <= numTimesAtLastPosition;
    } else {
      numTimesAtLastPosition = 0;
      return false;
    }
  }

  public void moveWrist(double power) {
     if (isSomethingStuck(power)) {
       power = 0;
     } 
    SmartDashboard.putNumber("Wrist power", power);
    wristMotor.set(ControlMode.PercentOutput, power, DemandType.ArbitraryFeedForward, calculateFeedForward());
  }

  public void  moveToPosition(double setPoint) {

    setPoint = angleToTics(setPoint);
    double kP = SmartDashboard.getNumber("Wrist P", 4);
    double kI = SmartDashboard.getNumber("Wrist I", 0);
    double kD = SmartDashboard.getNumber("Wrist D", 4);
    double kF = SmartDashboard.getNumber("Wrist F", 0.2);
    double curPose = getCurrentPosition();
//    if (curPose < setPoint) {
      wristMotor.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
      wristMotor.config_kF(Constants.kSlotIdx, kF, Constants.kTimeoutMs);
      wristMotor.config_kP(Constants.kSlotIdx, kP, Constants.kTimeoutMs);
      wristMotor.config_kI(Constants.kSlotIdx, kI, Constants.kTimeoutMs);
      wristMotor.config_kD(Constants.kSlotIdx, kD, Constants.kTimeoutMs);
    // } else {
    //   wristMotor.selectProfileSlot(Constants.kSlotDown, Constants.kPIDLoopIdx);
    //   wristMotor.config_kF(Constants.kSlotDown, kF / 100, Constants.kTimeoutMs);
    //   wristMotor.config_kP(Constants.kSlotDown, kP / 100, Constants.kTimeoutMs);
    //   wristMotor.config_kI(Constants.kSlotDown, kI, Constants.kTimeoutMs);
    //   wristMotor.config_kD(Constants.kSlotDown, kD, Constants.kTimeoutMs);
    // }
    wristMotor.set(ControlMode.MotionMagic, setPoint, DemandType.ArbitraryFeedForward, calculateFeedForward());
    SmartDashboard.putNumber("Wrist Vel", wristMotor.getActiveTrajectoryVelocity());
  }

  public int getCurrentPosition() {
    return ticsToAngle(wristMotor.getSelectedSensorPosition());
  }

  public void returnToManualControl() {
    moveWrist(0);
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

  public int ticsToAngle(double tics) {
    
    return ((int)Math.floor((tics / 1100) - lowestAngle));
  }
}
