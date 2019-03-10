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
import frc.robot.commands.ResetGyro;
import frc.robot.commands.ShiftUp;
import frc.robot.commands.WristMM;
import frc.robot.subsystems.Wrist;

public class Preload extends CommandGroup {
  /**
   * Add your docs here.
   */
  public Preload() {
    addParallel(new ResetGyro());
    addParallel(new ShiftUp());
    RobotMap.vision.toggleLimelight(true);
    addParallel(new BeakOpen());
    RobotMap.wristMotor.setSelectedSensorPosition(2784);
    addSequential(new WristMM(Wrist.Position.HIGH));
    addSequential(new WristMM(Wrist.Position.MIDDLE));
    addSequential(new BeakClose());
    addParallel(new WristMM(Wrist.Position.LOW));
    addSequential(new HoldAndWait());
  }
}
