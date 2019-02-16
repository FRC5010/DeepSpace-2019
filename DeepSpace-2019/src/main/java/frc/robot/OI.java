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
import frc.robot.commands.BeakOpen;
import frc.robot.commands.BeakClose;
import frc.robot.commands.commands_auto.FieldMovement;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  
  public class JoystickAxis {
    public final Joystick joystick;
    public final int axis;
    public double minPower = 0.15, maxPower = 1000;
    public boolean inversed = false;

    public JoystickAxis(Joystick pjoystick, int paxis) {
      joystick = pjoystick;
      axis = paxis;
    }

    public JoystickAxis(Joystick pjoystick, int paxis, double pminPower, double pmaxPower) {
      joystick = pjoystick;
      axis = paxis;
      minPower = pminPower;
      maxPower = pmaxPower;
    }

    public JoystickAxis(Joystick pjoystick, int paxis, boolean pinversed) {
      joystick = pjoystick;
      axis = paxis;
      inversed = pinversed;
    }

    public JoystickAxis(Joystick pjoystick, int paxis, double pminPower, double pmaxPower, boolean pinversed) {
      joystick = pjoystick;
      axis = paxis;
      minPower = pminPower;
      maxPower = pmaxPower;
      inversed = pinversed;
    }

    public double getValue() {
      return Math.max(scaleInputs(joystick.getRawAxis(axis)), maxPower);
    }

    // I don't think we use this function for anything else besides joysticks
    private double scaleInputs(double input) {
      if (Math.abs(input) < minPower) {
        input = 0;
      } else if (input > 0) {
        input = (input - minPower) * 1 / (1 - minPower);
      } else if (input < 0) {
        input = (input + minPower) * 1 / (1 - minPower);
      }
      return Math.pow(input, 3);
    }

    // "clamps" a value between the range min and max
    // private double clamp(double value, double min, double max) {
    //   double absValue = Math.abs(value);
    //   return absValue < max ? (absValue > min ? absValue : min) : max;
    // }
  }

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

  public JoystickAxis driveTrainForward;
  public JoystickAxis driveTrainTurn;
  public JoystickAxis elevatorLiftControl;
  public JoystickAxis ballIntake;
  public JoystickAxis ballOuttake;
  public JoystickAxis wristControl;

  public OI() {
    driverJoyLB.whenPressed(new ShiftDown());
    driverJoyRB.whenPressed(new ShiftUp());
    driverBack.whenPressed(new FieldMovement());
    driverLB.whenPressed(new BeakOpen());
    driverRB.whenPressed(new BeakClose());

    coDriverA.whenPressed(new ElevatorMM(0));
    coDriverB.whenPressed(new ElevatorMM(18000));

    // TODO: make sure axis number is correct!
    driveTrainForward = new JoystickAxis(driver, 1);
    driveTrainTurn = new JoystickAxis(driver, 4);
    elevatorLiftControl = new JoystickAxis(coDriver, 1);
    ballIntake = new JoystickAxis(coDriver, 2);
    ballOuttake = new JoystickAxis(coDriver, 3);
    wristControl = new JoystickAxis(coDriver, 5);
  }
}
