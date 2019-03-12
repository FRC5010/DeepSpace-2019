/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.RobotMap;
import frc.robot.commands.BeakClose;
import frc.robot.commands.BeakOpen;
import frc.robot.commands.HoldAndWait;
import frc.robot.commands.PreloadSetup;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.ShiftUp;
import frc.robot.commands.WristMM;
import frc.robot.subsystems.Wrist;

public class Preload extends CommandGroup {
  /**
   * Add your docs here.
   */

  public static boolean isPreloading = false;
  public Preload() {
    addSequential(new ResetGyro());
    addSequential(new ShiftUp());
    addSequential(new BeakOpen());
    addSequential(new PreloadSetup());
    addSequential(new WristMM(Wrist.Position.HIGH));
    addSequential(new WristMM(Wrist.Position.MIDDLE));
    addSequential(new BeakClose());
    addSequential(new WristMM(Wrist.Position.LOW));
    addSequential(new HoldAndWait());
  }
}
