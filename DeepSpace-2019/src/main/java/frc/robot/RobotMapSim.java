/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import frc.robot.subsystems.BallIntake;
import frc.robot.subsystems.BeakIntake;
import frc.robot.subsystems.DirectionSensor;
import frc.robot.subsystems.DistanceSensor;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shifter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.VisionAssistedDrive;

/**
 * Add your docs here.
 */
public class RobotMapSim {

    public static void initSim() {
        RobotMap.elevatorMotor = new TalonSRX(3);
        RobotMap.rightEncoder = new Encoder(0, 1);
        RobotMap.leftEncoder = new Encoder(2,3);
        RobotMap.encoderPPR=480;
        
        RobotMap.distance = new DistanceSensor();
        RobotMap.direction = new DirectionSensor(null);
        RobotMap.direction.reset();
    
        RobotMap.elevator = new Elevator();
        RobotMap.shifter = new Shifter();
        RobotMap.driveTrain = new DriveTrain();
        RobotMap.vision = new Vision();
        RobotMap.vision.changePipeline(0);
        RobotMap.visionDrive = new VisionAssistedDrive();
        RobotMap.ballIntake = new BallIntake();
        RobotMap.beakIntake = new BeakIntake();
      }
}



