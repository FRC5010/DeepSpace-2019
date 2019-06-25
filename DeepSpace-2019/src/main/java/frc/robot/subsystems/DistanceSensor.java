package frc.robot.subsystems;

import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DistanceSensor extends Subsystem {
	
	double rightDPP = 0.0032725;
	//right encoder reversed?
	
	
	public DistanceSensor() {
		RobotMap.rightEncoder.reset();
		RobotMap.leftEncoder.reset();
		
		RobotMap.rightEncoder.setDistancePerPulse(rightDPP);
		RobotMap.leftEncoder.setDistancePerPulse(rightDPP);
		
		RobotMap.rightEncoder.setReverseDirection(true);		
	}

	public DistanceSensor(String name) {
		super(name);
	}
	public double getDistance() {
		SmartDashboard.putNumber("right encoder getRaw", RobotMap.rightEncoder.getRaw());
		
		SmartDashboard.putNumber("left encoder getRaw", RobotMap.leftEncoder.getRaw());
		return RobotMap.rightEncoder.getDistance();
	}
	
	public int getLeftRaw() {
		SmartDashboard.putNumber("left encoder getRaw", RobotMap.leftEncoder.getRaw());
		return RobotMap.leftEncoder.getRaw();
	}
	
	public int getRightRaw() {
		SmartDashboard.putNumber("right encoder getRaw", RobotMap.rightEncoder.getRaw());
		return RobotMap.rightEncoder.getRaw();
		
	}
	public void reset() {
		RobotMap.rightEncoder.reset();
		RobotMap.leftEncoder.reset();
	}
	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
	}

}
