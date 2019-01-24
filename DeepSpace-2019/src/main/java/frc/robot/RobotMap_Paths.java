/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

//import com.ctre.phoenix.motion.TrajectoryPoint;

//import java.nio.file.DirectoryStream;

import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;

/**
 * Add your docs here.
 */
public class RobotMap_Paths {
 public static Trajectory testL;
public static Trajectory testR;
public static Trajectory drive1R;
public static Trajectory drive1L;
public static Trajectory revR;
public static Trajectory revL;
public static Trajectory drive2R;
public static Trajectory drive2L;
public static Trajectory drive3R;
public static Trajectory drive3L;
public static Trajectory driveFinR;
public static Trajectory driveFinL;

    
public static void init(){
     testL = PathfinderFRC.getTrajectory("testPath.left" );
     System.out.println("testL"+testL.toString());
     testR = PathfinderFRC.getTrajectory("testPath.right");
     System.out.println("testR"+testR.toString());
     drive1L = PathfinderFRC.getTrajectory("testPath.right");
     drive1R = PathfinderFRC.getTrajectory("testPath.right");
     revR = PathfinderFRC.getTrajectory("testPath.right");
     revL = PathfinderFRC.getTrajectory("testPath.right");
     drive2R = PathfinderFRC.getTrajectory("testPath.right");
     drive2L = PathfinderFRC.getTrajectory("testPath.right");
     drive3L = PathfinderFRC.getTrajectory("testPath.right");
     drive3R = PathfinderFRC.getTrajectory("testPath.right");
     driveFinR = PathfinderFRC.getTrajectory("testPath.right");
     driveFinL = PathfinderFRC.getTrajectory("testPath.right");

}
}
