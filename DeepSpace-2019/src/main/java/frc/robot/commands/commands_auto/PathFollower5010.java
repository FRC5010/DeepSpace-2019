/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.commands_auto;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.RobotMap_Paths;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;

public class PathFollower5010 extends Command {

	EncoderFollower5010 left, right;
	public static final boolean kForward = true;
	public static final boolean kReverse = false;
	private boolean isFwd = true;

	public PathFollower5010(Trajectory lTraj, Trajectory rTraj, boolean isFwd) {
		requires(RobotMap.driveTrain);
		requires(RobotMap.direction);
		requires(RobotMap.distance);
		this.isFwd = isFwd;
		// Swapped because of Pathweaver issues for the time being.
		// TODO: REVERSED WHEN FIXED!!!!!!!!!!!!
		if (isFwd) {
			left = new EncoderFollower5010(rTraj, false, isFwd);
			right = new EncoderFollower5010(lTraj, true, isFwd);
		} else {
			left = new EncoderFollower5010(lTraj, true, isFwd);
			right = new EncoderFollower5010(rTraj, false, isFwd);
		}
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		SmartDashboard.putBoolean("Running", true);

		// Encoder Position is the current, cumulative position of your encoder. If
		// you're using an SRX, this will be the
		// 'getEncPosition' function.
		// Second param is the amount of encoder ticks per full revolution
		// Wheel Diameter is the diameter of your wheels (or pulley for a track system)
		// in the chosen units
		left.reset();
		right.reset();
		left.configureEncoder(RobotMap.distance.getLeftRaw(), RobotMap.encoderPPR, RobotMap_Paths.wheel_diameter);
		right.configureEncoder(RobotMap.distance.getRightRaw(), RobotMap.encoderPPR, RobotMap_Paths.wheel_diameter);

		// The first argument is the proportional gain. Usually this will be quite high
		double kp = 1.0;
		// The second argument is the integral gain. This is unused for motion profiling
		// The third argument is the derivative gain. Tweak this if you are unhappy with
		// the tracking of the trajectory
		double kd = 0.01;
		// The fourth argument is the velocity ratio. This is 1 over the maximum
		// velocity you provided in the trajectory configuration (it translates m/s to a
		// -1 to 1 scale that your
		// motors can read)
		double kv = 1.0 / RobotMap_Paths.max_velocity;
		// The fifth argument is your acceleration gain. Tweak this if you want to get
		// to a higher or lower speed quicker
		double ka = 0.1;
		// Sixth - The position error tolerance to achieve before isFinished will return
		// true
		double ket = .1;
		// Seventh - The heading error tolerance to achieve before isFinished will
		// return true
		double kht = 1;
		left.configurePIDVA(kp, 0.0, kd, kv, ka, ket, kht);
		right.configurePIDVA(kp, 0.0, kd, kv, ka, ket, kht);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
  protected void execute() {
	double gyro_heading = RobotMap.direction.angle();
	if(isFwd) {
		gyro_heading =  -gyro_heading;
	}
	double l = left.calculate(RobotMap.distance.getLeftRaw(), gyro_heading);
	double r = right.calculate(RobotMap.distance.getRightRaw(), gyro_heading);
		
	Trajectory.Segment lseg = left.getSegment();
	Trajectory.Segment rseg = right.getSegment();
    SmartDashboard.putNumber("left accel", lseg.acceleration);
	SmartDashboard.putNumber("left time delta", lseg.dt);
	//TODO: REVERSE NEGATION AFTER PATHWEAVER FIX.
	SmartDashboard.putNumber("left heading", -Pathfinder.r2d(lseg.heading));
	SmartDashboard.putNumber("left jerk", lseg.jerk);
	SmartDashboard.putNumber("left pos", lseg.position);
	SmartDashboard.putNumber("left velocity", lseg.velocity);
	SmartDashboard.putNumber("left x", lseg.x);
	SmartDashboard.putNumber("left y", lseg.y);
		
	SmartDashboard.putNumber("right accel", rseg.acceleration);
	//TODO: REVERSE NEGATION AFTER PATHWEAVER FIX.
	SmartDashboard.putNumber("right time delta", rseg.dt);
	SmartDashboard.putNumber("right heading", rseg.heading);
	SmartDashboard.putNumber("right jerk", rseg.jerk);
	SmartDashboard.putNumber("right pos", rseg.position);
	SmartDashboard.putNumber("right velocity", rseg.velocity);
	SmartDashboard.putNumber("right x", rseg.x);
	SmartDashboard.putNumber("right y", rseg.y);
	double distance_covered = ((double)(RobotMap.distance.getLeftRaw() - 0) / RobotMap.encoderPPR) * RobotMap_Paths.wheel_diameter;
	SmartDashboard.putNumber("distance covered", distance_covered);

	//TODO: REVERSE NEGATION AFTER PATHWEAVER FIX!!!!!
	double desired_heading = -Pathfinder.r2d(lseg.heading); // Should also be in degrees
	//TODO: REVERSE NEGATION AFTER PATHWEAVER UPDATE
	SmartDashboard.putNumber("desired Heading", -desired_heading);

	SmartDashboard.putNumber("left output", l);
	SmartDashboard.putNumber("right output", r);
	RobotMap.driveTrain.drive(l, r);
  }

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return left.isFinished() || right.isFinished();
	}

	// Called once after isFinished retu0rns true
	@Override
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
	}
}
