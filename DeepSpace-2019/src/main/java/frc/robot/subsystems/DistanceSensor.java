package frc.robot.subsystems;

import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DistanceSensor extends Subsystem {
	
	double rightDPP = .16;//1 / 25.4; //.039;
	//right encoder reversed?
	
	
	public DistanceSensor() {
		RobotMap.rightEncoder.reset();
		RobotMap.leftEncoder.reset();
		
		double dpp = RobotMap.rightEncoder.getDistancePerPulse();
		SmartDashboard.getNumber("Distance per pulse", dpp);
		
		
		
		RobotMap.rightEncoder.setDistancePerPulse(rightDPP);
		RobotMap.leftEncoder.setDistancePerPulse(rightDPP);
		
		dpp = RobotMap.rightEncoder.getDistancePerPulse();
		RobotMap.rightEncoder.setReverseDirection(true);
		SmartDashboard.getNumber("Distance per pulse r", dpp);
		
		dpp = RobotMap.leftEncoder.getDistancePerPulse();
		SmartDashboard.getNumber("Distance per pulse l", dpp);
		
	}

	public DistanceSensor(String name) {
		super(name);
	}
	public double getDistance() {
		SmartDashboard.putNumber("right encoder getRaw", RobotMap.rightEncoder.getRaw());
		
		SmartDashboard.putNumber("left encoder getRaw", RobotMap.leftEncoder.getRaw());
		SmartDashboard.putNumber("distance",RobotMap.rightEncoder.getDistance());
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
