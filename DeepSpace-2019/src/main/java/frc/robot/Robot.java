/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.WristMM;
import frc.robot.commands.groups.DriveOffHABLevel1;
import frc.robot.commands.groups.LeftRocket;
import frc.robot.commands.groups.LeftShipFrontAndSide;
import frc.robot.commands.groups.LeftShipSideX2;
import frc.robot.commands.groups.MiddleShipLeft;
import frc.robot.commands.groups.MiddleShipRight;
import frc.robot.commands.groups.RightRocket;
import frc.robot.commands.groups.RightShipFrontAndSide;
import frc.robot.commands.groups.RightShipSideX2;
import frc.robot.subsystems.Pose;
import frc.robot.subsystems.Wrist.Position;
import frc.robot.util.Instrum;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static OI oi;

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    RobotMap.init();
    oi = new OI();
    
    m_chooser.setDefaultOption("HAB Level 1", new DriveOffHABLevel1());
    m_chooser.setDefaultOption("Left Rocket", new LeftRocket());
    m_chooser.setDefaultOption("Left Ship Front+Side", new LeftShipFrontAndSide());
    m_chooser.setDefaultOption("Left Ship SideX2", new LeftShipSideX2());
    m_chooser.setDefaultOption("Middle Ship Left", new MiddleShipLeft());
    m_chooser.setDefaultOption("Middle Ship Right", new MiddleShipRight());
    m_chooser.setDefaultOption("Right Rocket", new RightRocket());
    m_chooser.setDefaultOption("Right Ship Front+Side", new RightShipFrontAndSide());
    m_chooser.setDefaultOption("Right Ship SideX2", new RightShipSideX2());

    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);

    if (DriverStation.getInstance().getMatchType() != MatchType.None) {
      new WristMM(Position.PRELOAD).start();;
    }
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    Pose.update(RobotController.getFPGATime());
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_chooser.getSelected();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    if (oi.driveTrainForward.getValue() != 0 || oi.driveTrainTurn.getValue() != 0) {
      m_autonomousCommand.cancel();
    }
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    Instrum.Process(RobotMap.elevatorMotor, new StringBuilder());
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
