/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.RobotMap_Paths.MotionProfiles;
import frc.robot.commands.LimeLightState;
import frc.robot.commands.ShiftUp;
import frc.robot.commands.LimeLightState.State;
import frc.robot.commands.commands_auto.PathFollower5010;
import frc.robot.commands.commands_auto.PathFollower5010.Direction;

public class RightShipSideX2 extends CommandGroup {
  /**
   * Add your docs here.
   */
  public RightShipSideX2() {
    addSequential(new Preload());
    //addSequential(new DriveOffHABLevel2Sequence());

    // Right start to Ship right-side first bay
    addSequential(new ShiftUp());
    addSequential(new PathFollower5010(MotionProfiles.RStoShip2R, Direction.kForward));
    addParallel(new LimeLightState(State.DRIVER));
    //addSequential(new VisionReleaseHatch());
   // addSequential(new PathFollower5010(MotionProfiles.backupShip2R, Direction.kRevNormal));
    
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
  }
}
