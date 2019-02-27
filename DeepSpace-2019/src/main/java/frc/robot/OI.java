/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.BeakClose;
import frc.robot.commands.BeakOpen;
import frc.robot.commands.ElevatorMM;
import frc.robot.commands.ElevatorReset;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.ShiftDown;
import frc.robot.commands.ShiftUp;
import frc.robot.commands.ToggleMotorSafety;
import frc.robot.commands.VADriveUntilDistance;
import frc.robot.commands.WristMM;
import frc.robot.commands.WristReset;
import frc.robot.commands.commands_auto.FieldMovement;
import frc.robot.commands.groups.VisionGrabHatch;
import frc.robot.dynasty.JoystickAxis;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.Position;
import frc.robot.subsystems.Wrist;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  public Joystick driver = new Joystick(0);
  public Joystick coDriver = new Joystick(1);

  public Button driverLB = new JoystickButton(driver, 5);
  public Button driverRB = new JoystickButton(driver, 6);
  public Button driverBack = new JoystickButton(driver, 7);
  public Button driverStart = new JoystickButton(driver, 8);
  public Button driverJoyLB = new JoystickButton(driver, 9);
  public Button driverJoyRB = new JoystickButton(driver, 10);
  public Button driverA = new JoystickButton(driver, 1);
  public Button driverB = new JoystickButton(driver, 2);
  public Button driverX = new JoystickButton(driver, 3);
  public Button driverY = new JoystickButton(driver, 4);

  public Button coDriverA = new JoystickButton(coDriver, 1);
  public Button coDriverB = new JoystickButton(coDriver, 2);
  public Button coDriverX = new JoystickButton(coDriver, 3);
  public Button coDriverY = new JoystickButton(coDriver, 4);
  public Button coDriverLB = new JoystickButton(coDriver, 5);
  public Button coDriverRB = new JoystickButton(coDriver, 6);
  public Button coDriverBack = new JoystickButton(coDriver, 7);
  public Button coDriverStart = new JoystickButton(coDriver,8);
  public Button coDriverJoyLB = new JoystickButton(coDriver, 9);
  public Button coDriverJoyRB = new JoystickButton(coDriver, 10);

  public Button elevatorGamePieceSelector;

  public JoystickAxis driveTrainForward;
  public JoystickAxis driveTrainTurn;

  public JoystickAxis visionActivation;
  public JoystickAxis elevatorLiftControl;
  public JoystickAxis ballIntake;
  public JoystickAxis ballOuttake;
  public JoystickAxis wristControl;

  public OI() {
    /** Driver controls */
    driverStart.whenPressed(new ResetGyro());
    driverBack.whenPressed(new FieldMovement());

    driverJoyLB.whenPressed(new ShiftDown());
    driverJoyRB.whenPressed(new ShiftUp());
    
    driverLB.whenPressed(new BeakOpen());
    driverRB.whenPressed(new BeakClose());
    
    driverA.whenPressed(new WristMM(Wrist.CARGO_LOW));
    driverB.whenPressed(new WristMM(Wrist.CARGO_MIDDLE));

    driverX.whenPressed(new VisionGrabHatch());
    //driverY.whenPressed(new VADriveUntilDistance());
    
    driveTrainForward = new JoystickAxis(driver, 1, true, 0.7);
    visionActivation = new JoystickAxis(driver, 3);
    driveTrainTurn = new JoystickAxis(driver, 4, 0.5);
    
    /** Co-Driver controls */
    coDriverStart.whenPressed(new ElevatorReset());
    coDriverBack.whenPressed(new WristReset());
    coDriverJoyLB.whenPressed(new ToggleMotorSafety());

    coDriverA.whenPressed(new ElevatorMM(Position.LOW));
    coDriverB.whenPressed(new ElevatorMM(Position.MIDDLE));
    coDriverY.whenPressed(new ElevatorMM(Position.HIGH));

    elevatorLiftControl = new JoystickAxis(coDriver, 1, true, Elevator.MAX_FWD_OUT);
    elevatorLiftControl.setLowerLimit(Elevator.MAX_REV_OUT);
    elevatorGamePieceSelector = coDriverLB;

    ballIntake = new JoystickAxis(coDriver, 2, 0.3);
    ballIntake.setLowerLimit(0);
    ballOuttake = new JoystickAxis(coDriver, 3, true, 0.3);
    ballOuttake.setUpperLimit(0);

    wristControl = new JoystickAxis(coDriver, 5, true, Wrist.MAX_FWD_OUT);
    wristControl.setLowerLimit(Wrist.MAX_REV_OUT);
  }
}
