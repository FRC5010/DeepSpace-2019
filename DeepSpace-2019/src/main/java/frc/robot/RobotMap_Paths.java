/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;

//import com.ctre.phoenix.motion.TrajectoryPoint;

//import java.nio.file.DirectoryStream;

import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;

/**
 * Add your docs here.
 */
public class RobotMap_Paths {
     public static final double max_velocity = 17.89;
     public static final double wheel_diameter = 0.5;
     public static Map<String, Trajectory> leftTrajectories;
     public static Map<String, Trajectory> rightTrajectories;

     // TODO: Change all Trajectories to Strings with just the base name
     public static String MStoShip1L = "MStoShip1L";

     public static Trajectory testL;
     public static Trajectory testR;
     public static Trajectory exit_level_two_left;
     public static Trajectory exit_level_two_right;

     public static Trajectory LStoShip2L_left;
     public static Trajectory LStoShip2L_right;
     public static Trajectory Ship2LtoLP_right;
     public static Trajectory Ship2LtoLP_left;
     public static Trajectory MStoShip1R_left;
     public static Trajectory MStoShip1R_right;
     public static Trajectory RStoShip1R_left;
     public static Trajectory RStoShip1R_right;
     public static Trajectory left_ship_to_1L_left;
     public static Trajectory left_ship_to_1L_right;
     public static Trajectory right_ship_to_2R_left;
     public static Trajectory right_ship_to_2R_right;
     public static Trajectory backUp_ship_1L_left;
     public static Trajectory backUp_ship_1L_right;
     public static Trajectory backUp_ship_1R_left;
     public static Trajectory backUp_ship_1R_right;
     public static Trajectory backUp_ship_2L_left;
     public static Trajectory backUp_ship_2L_right;
     public static Trajectory backUp_ship_2R_left;
     public static Trajectory backUp_ship_2R_right;
     public static Trajectory Ship1RtoRP_right;
     public static Trajectory Ship1RtoRP_left;
     public static Trajectory lsR_to_Right_2R_right;
     public static Trajectory lsR_to_Right_2R_left;

     private static boolean errorLoadingPaths = false;

     public static void init() {
          List<String> trajectories = new ArrayList<>();
          leftTrajectories = new HashMap<>();
          rightTrajectories = new HashMap<>();

          // TODO: Add the trajectory basename String to the list
          trajectories.add(MStoShip1L);

          try {
               for (String trajectory : trajectories) {
                    loadTrajectories(trajectory);
               }
          
          // TODO: REMOVE the old Trajectory loading code.

          // MiddleShipRight
          MStoShip1R_left = PathfinderFRC.getTrajectory("MStoShip1R.left");
          System.gc();
          System.out.println("MStoShip1R.left: "+ MStoShip1R_left.length());
          MStoShip1R_right = PathfinderFRC.getTrajectory("MStoShip1R.right");
          System.gc();
          System.out.println("MStoShip1R.right: "+ MStoShip1R_right.length());

          // RightShipFrontAndSide
          RStoShip1R_left = PathfinderFRC.getTrajectory("RStoShip1R.left");
          System.gc();
          System.out.println("RStoShip1R.left: "+ RStoShip1R_left.length());
          RStoShip1R_right = PathfinderFRC.getTrajectory("RStoShip1R.right");
          System.gc();
          System.out.println("RStoShip1R.right: "+ RStoShip1R_right.length());

          backUp_ship_1R_left = PathfinderFRC.getTrajectory("backUp_ship_1R.left");
          System.gc();
          System.out.println("backUp_ship_1R_left"+ backUp_ship_1R_left.toString());
          backUp_ship_1R_right = PathfinderFRC.getTrajectory("backUp_ship_1R.right");
          System.gc();
          System.out.println("backUp_ship_1R_right"+ backUp_ship_1R_right.toString());

          Ship1RtoRP_left= PathfinderFRC.getTrajectory("Ship1RtoRP.left");
          System.gc();
          System.out.println("Ship1RtoRP.left "+ Ship1RtoRP_left.toString());
          Ship1RtoRP_right = PathfinderFRC.getTrajectory("Ship1RtoRP.right");
          System.gc();
          System.out.println("Ship1RtoRP.right "+ Ship1RtoRP_right.toString());

          // RightShipSideX2
          right_ship_to_2R_left = PathfinderFRC.getTrajectory("right_ship_to_2R.left");
          System.gc();
          System.out.println("right_ship_to_2R_left"+ right_ship_to_2R_left.toString());
          right_ship_to_2R_right = PathfinderFRC.getTrajectory("right_ship_to_2R.right");
          System.gc();
          System.out.println("right_ship_to_2R_right"+ right_ship_to_2R_right.toString());

          backUp_ship_2R_left = PathfinderFRC.getTrajectory("backUp_ship_2R.left");
          System.gc();
          System.out.println("backUp_ship_2R_left"+ backUp_ship_2R_left.toString());
          backUp_ship_2R_right = PathfinderFRC.getTrajectory("backUp_ship_2R.right");
          System.gc();
          System.out.println("backUp_ship_2R_right"+ backUp_ship_2R_right.toString());

          lsR_to_Right_2R_left = PathfinderFRC.getTrajectory("lsR_to_Right_ship2R.left");
          System.gc();
          System.out.println("lsR_to_Right_2R_left"+ lsR_to_Right_2R_left.toString());
          lsR_to_Right_2R_right = PathfinderFRC.getTrajectory("lsR_to_Right_ship2R.right");
          System.gc();
          System.out.println("lsR_to_Right_2R_right"+ lsR_to_Right_2R_right.toString());

          // LeftShipFrontAndSide
          left_ship_to_1L_left = PathfinderFRC.getTrajectory("left_ship_to_1L.left");
          System.gc();
          System.out.println("left_ship_to_1L_left"+ left_ship_to_1L_left.toString());
          left_ship_to_1L_right = PathfinderFRC.getTrajectory("left_ship_to_1L.right");
          System.gc();
          System.out.println("left_ship_to_1L_right"+ left_ship_to_1L_right.toString());

          backUp_ship_1L_left = PathfinderFRC.getTrajectory("backUp_ship_1L.left");
          System.gc();
          System.out.println("backUp_ship_1L_left"+ backUp_ship_1L_left.toString());
          backUp_ship_1L_right = PathfinderFRC.getTrajectory("backUp_ship_1L.right");
          System.gc();
          System.out.println("backUp_ship_1L_right"+ backUp_ship_1L_right.toString());

          // LeftShipSideX2
          LStoShip2L_left = PathfinderFRC.getTrajectory("LStoShip2L.left");
          System.gc();
          System.out.println("LStoShip2L.left: "+ backUp_ship_2R_right.length());
          LStoShip2L_left = PathfinderFRC.getTrajectory("LStoShip2L.right");
          System.gc();
          System.out.println("LStoShip2L.right: "+ backUp_ship_2R_right.length());

          backUp_ship_2L_left = PathfinderFRC.getTrajectory("backUp_ship_2L.left");
          System.gc();
          System.out.println("backUp_ship_2L_left"+ backUp_ship_2L_left.toString());
          backUp_ship_2L_right = PathfinderFRC.getTrajectory("backUp_ship_2L.right");
          System.gc();
          System.out.println("backUp_ship_2L_right"+ backUp_ship_2L_right.toString());

          Ship2LtoLP_right = PathfinderFRC.getTrajectory("Ship2LtoLP.right");
          System.gc();
          System.out.println("backUp_ship_2R_right"+ backUp_ship_2R_right.toString());
          Ship2LtoLP_left = PathfinderFRC.getTrajectory("Ship2LtoLP.left");
          System.gc();
          System.out.println("backUp_ship_2R_right"+ backUp_ship_2R_right.toString());


          // Suicide run
          exit_level_two_left = PathfinderFRC.getTrajectory("exit_level_two.left");
          System.gc();
          System.out.println("exit_level_two_left"+exit_level_two_left.toString());
          exit_level_two_right = PathfinderFRC.getTrajectory("exit_level_two.right");
          System.gc();
          System.out.println("exit_level_two_right"+exit_level_two_right.toString());

          testL = PathfinderFRC.getTrajectory("testPath.left");
          System.gc();
          System.out.println("testL" + testL.toString());
          testR = PathfinderFRC.getTrajectory("testPath.right");
          System.gc();
          System.out.println("testR" + testR.toString());

          } catch (IOException e) {
               System.err.println("There was a problem loading paths, fix it! " + e.getMessage());
          }
     }

     private static void loadTrajectories(String basePathName) {
          String absolutePath = Filesystem.getDeployDirectory().getAbsolutePath();
          String leftPath = absolutePath + "\\paths\\" + basePathName + ".left.pf1.csv";
          String rightPath = absolutePath + "\\paths\\" + basePathName + ".right.pf1.csv";
          File leftFile = new File(leftPath);
          File rightFile = new File(rightPath);
          if (!errorLoadingPaths) {
               SmartDashboard.putString("Motion Profile Path Status", "YEET!");
          }
          try {
               leftTrajectories.put(basePathName, Pathfinder.readFromCSV(leftFile));
               System.out.println(leftPath + ": " + leftTrajectories.get(basePathName).length());
               rightTrajectories.put(basePathName, Pathfinder.readFromCSV(rightFile));
               System.out.println(rightPath + ": " + rightTrajectories.get(basePathName).length());
          } catch (IOException e) {
               errorLoadingPaths = true;
               System.err.println("******************* AAAAAAAAAAAAAAAAAAAAAAAAAHHHHHHHH!!!!!!!!!!!!! *****************");
               System.err.println("***   " + e.getMessage());
               System.err.println("******************* AAAAAAAAAAAAAAAAAAAAAAAAAHHHHHHHH!!!!!!!!!!!!! *****************");
               SmartDashboard.putString("Motion Profile Path Status", e.getMessage());
          }
     }
}
