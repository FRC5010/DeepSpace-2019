package frc.robot.subsystems;

import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.interfaces.Gyro;

public class DirectionSensor extends Subsystem {

	private Gyro gyro;
	
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

	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}
	
	
}
