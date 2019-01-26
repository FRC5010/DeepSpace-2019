/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.commands_auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.RobotMap_Paths;

public class FieldMovement extends CommandGroup {
  /**
   * Add your docs here.
   */
  public FieldMovement() {
    addSequential(new PathForward(RobotMap_Paths.testL,RobotMap_Paths.testR));
    // addSequential(new PathForward(RobotMap_Paths.drive1L,RobotMap_Paths.drive1R));
    // addSequential(new PathReverse(RobotMap_Paths.revL,RobotMap_Paths.revR));
    // addSequential(new PathForward(RobotMap_Paths.drive2L,RobotMap_Paths.drive2R));
    // addSequential(new PathReverse(RobotMap_Paths.drive3L,RobotMap_Paths.drive3R));
    // addSequential(new PathForward(RobotMap_Paths.driveFinL,RobotMap_Paths.driveFinR));
    // // Add Commands here:
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
