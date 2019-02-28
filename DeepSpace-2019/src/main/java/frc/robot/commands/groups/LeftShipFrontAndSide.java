/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.RobotMap_Paths;
import frc.robot.commands.commands_auto.PathFollower5010;
import frc.robot.commands.commands_auto.PathFollower5010.Direction;

public class LeftShipFrontAndSide extends CommandGroup {
  /**
   * Add your docs here.
   */
  public LeftShipFrontAndSide() {
    addSequential(new Preload());

    // Left start to Ship left-front bay, then backup, then to loading station, then to left side first bay
    addSequential(new PathFollower5010(RobotMap_Paths.left_ship_to_1L_left, RobotMap_Paths.left_ship_to_1L_right, Direction.kForward));
    addSequential(new PathFollower5010(RobotMap_Paths.backUp_ship_1L_left, RobotMap_Paths.backUp_ship_1L_right, Direction.kRevNormal));

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
