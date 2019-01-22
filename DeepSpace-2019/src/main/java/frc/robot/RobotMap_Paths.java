/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.nio.file.DirectoryStream;

import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;

/**
 * Add your docs here.
 */
public class RobotMap_Paths {
 public static Trajectory testL;
public static Trajectory testR;

    
public static void init(){
     testL = PathfinderFRC.getTrajectory("Unnamed.left");
     System.out.println("testL"+testL.toString());
     testR = PathfinderFRC.getTrajectory("Unnamed.right");
     System.out.println("testT"+testR.toString());
}
}
