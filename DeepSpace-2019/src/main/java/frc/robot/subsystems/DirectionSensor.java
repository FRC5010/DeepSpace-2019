/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class DirectionSensor extends Subsystem {
  
  private Gyro gyro;
// this is to reset the gyro so it can detect the next angle
  public DirectionSensor() {
    RobotMap.gyro.reset();
    this.gyro = RobotMap.gyro;
  }
  
  public double angle() {
		return gyro.getAngle();
	}
	
	public void reset() {
		gyro.reset();
	}
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
