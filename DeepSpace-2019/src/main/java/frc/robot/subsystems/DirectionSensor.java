/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class DirectionSensor extends Subsystem {

  private AHRS gyro;
  private double initialOffset = 0.0;

  // this is to reset the gyro so it can detect the next angle
  public DirectionSensor(AHRS gyro) {
    this.gyro = gyro;
    if (null != gyro) {
      gyro.reset();
    }
  }

  public DirectionSensor(double initialOffset) {
    RobotMap.gyro.reset();
    this.gyro = RobotMap.gyro;
    this.initialOffset = initialOffset;
  }

  public double angle() {
    if (null != gyro) {
      SmartDashboard.putNumber("gyro", gyro.getAngle() + initialOffset);
      return -(gyro.getAngle() + initialOffset);
    }
    return 0;
  }

  public static double boundHalfDegrees(double angle_degrees) {
    while (angle_degrees >= 180.0)
      angle_degrees -= 360.0;
    while (angle_degrees < -180.0)
      angle_degrees += 360.0;
    return angle_degrees;
  }

  public void reset() {
    if (null != gyro) {
      gyro.reset();
    }
  }
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
