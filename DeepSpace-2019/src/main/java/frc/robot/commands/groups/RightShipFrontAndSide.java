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

public class RightShipFrontAndSide extends CommandGroup {
  /**
   * Add your docs here.
   */
  public RightShipFrontAndSide() {
    addSequential(new Preload());

    // Right start to Ship right-front bay then backup
    addSequential(new PathFollower5010(RobotMap_Paths.RStoShip1R_left, RobotMap_Paths.RStoShip1R_right, Direction.kForward));
    addSequential(new VisionReleaseHatch());
    addSequential(new PathFollower5010(RobotMap_Paths.backUp_ship_1R_left, RobotMap_Paths.backUp_ship_1R_right, Direction.kRevNormal));
    addSequential(new PathFollower5010(RobotMap_Paths.Ship1RtoRP_left, RobotMap_Paths.Ship1RtoRP_right, Direction.kForward));
    addSequential(new VisionGrabHatch());
    addSequential(new PathFollower5010(RobotMap_Paths.lsR_to_Right_2R_left,RobotMap_Paths.lsR_to_Right_2R_right,Direction.kRevFlipped));
    //addSequentional(new PathFollower5010(RobotMap_Paths.finalShip2R));    
    //addSequential(new VisionReleaseHatch());

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
