/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.BeakClose;
import frc.robot.commands.BeakOpen;
import frc.robot.commands.PreloadFinish;
import frc.robot.commands.PreloadSetup;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.ShiftUp;
import frc.robot.commands.WristMM;
import frc.robot.commands.spinWheels;
import frc.robot.subsystems.Wrist;

public class Preload extends CommandGroup {
  /**
   * Add your docs here.
   */

  public static boolean isPreloading = false;
  public Preload() {
    addSequential(new PreloadSetup());
    addSequential(new ResetGyro());
    addSequential(new BeakOpen());
    addSequential(new WristMM(Wrist.Position.HIGH));
    
    addParallel(new WristMM(Wrist.Position.MIDDLE));
    addParallel(new BeakClose());
    addSequential(new ShiftUp());
    //addSequential(new spinWheels());
    //addParallel(new BeakClose());
    addSequential(new WristMM(Wrist.Position.LOW));
    addSequential(new PreloadFinish());
  }
}
