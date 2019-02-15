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
import frc.robot.commands.ElevatorMM;
import frc.robot.commands.ShiftDown;
import frc.robot.commands.ShiftUp;
import frc.robot.commands.WristMM;
import frc.robot.commands.BeakOpen;
import frc.robot.commands.BeakClose;
import frc.robot.commands.commands_auto.FieldMovement;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());
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

  public Button coDriverA = new JoystickButton(coDriver, 1);
  public Button coDriverB = new JoystickButton(coDriver, 2);
  public Button coDriverX = new JoystickButton(coDriver, 3);
  public Button coDriverY = new JoystickButton(coDriver, 4);
  public Button coDriverLB = new JoystickButton(coDriver, 5);
  public Button coDriverRB = new JoystickButton(coDriver, 6);

  private static double deadZone = 0.15;

  public OI() {
    driverJoyLB.whenPressed(new ShiftDown());
    driverJoyRB.whenPressed(new ShiftUp());
    driverBack.whenPressed(new FieldMovement());
    driverLB.whenPressed(new BeakOpen());
    driverRB.whenPressed(new BeakClose());
    driverB.whileHeld(new WristMM(18000));
    driverA.whileHeld(new WristMM(0));

    coDriverA.whenPressed(new ElevatorMM(0));
    coDriverB.whenPressed(new ElevatorMM(18000));
  }

  public static double scaleInputs(double input) {
    if (Math.abs(input) < deadZone) {
      input = 0;
    } else if (input > 0) {
      input = (input - deadZone) * 1 / (1 - deadZone);
    } else if (input < 0) {
      input = (input + deadZone) * 1 / (1 - deadZone);
    }
    return Math.pow(input, 3);
  }

  // TODO: check and make sure axis parameters are correct

  public double getLeftJoystickForward (Joystick joystick) {
    return scaleInputs(joystick.getRawAxis(1));
  }

  public double getRightJoystickForward (Joystick joystick) {
    return scaleInputs(joystick.getRawAxis(5));
  }

  public double getLeftJoystickHorizontal (Joystick joystick) {
    return scaleInputs(joystick.getRawAxis(6));
  }

  public double getRightJoystickHorizontal (Joystick joystick) {
    return scaleInputs(joystick.getRawAxis(4));
  }

  public double getLeftTrigger (Joystick joystick) {
    return scaleInputs(joystick.getRawAxis(2));
  }
  
  public double getRightTrigger(Joystick joystick) {
    return scaleInputs(joystick.getRawAxis(3));
  }
}
