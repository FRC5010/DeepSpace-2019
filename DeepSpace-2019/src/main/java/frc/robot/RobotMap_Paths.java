/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;

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
     
     public static Trajectory testL;
     public static Trajectory testR;
     public static Trajectory drive1R;
     public static Trajectory drive1L;
     public static Trajectory revLeftR;
     public static Trajectory revLeftL;
     public static Trajectory revRightR;
     public static Trajectory revRightL;
     public static Trajectory drive2R;
     public static Trajectory drive2L;
     public static Trajectory drive3R;
     public static Trajectory drive3L;
     public static Trajectory driveFinR;
     public static Trajectory driveFinL;
     public static Trajectory exit_level_two_left;
     public static Trajectory exit_level_two_right;
     public static Trajectory mid_ship_to_1R_left;
     public static Trajectory mid_ship_to_1R_right;
     public static Trajectory mid_ship_to_1L_left;
     public static Trajectory mid_ship_to_1L_right;
     public static Trajectory right_ship_to_1R_left;
     public static Trajectory right_ship_to_1R_right;
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
     public static Trajectory R1_to_lsR_right;
     public static Trajectory R1_to_lsR_left;
     public static Trajectory lsR_to_Right_2R_right;
     public static Trajectory lsR_to_Right_2R_left;



     public static void init() {
          try {
          mid_ship_to_1L_left = PathfinderFRC.getTrajectory("mid_ship_to_1L.left");
          System.out.println("mid_ship_to_1L_left"+ mid_ship_to_1L_left.toString());
          mid_ship_to_1L_right = PathfinderFRC.getTrajectory("mid_ship_to_1L.right");
          System.out.println("mid_ship_to_1L_right"+ mid_ship_to_1L_right.toString());
          
          mid_ship_to_1R_left = PathfinderFRC.getTrajectory("mid_ship_to_1R.left");
          System.out.println("mid_ship_to_1R_left"+ mid_ship_to_1R_left.toString());
          mid_ship_to_1R_right = PathfinderFRC.getTrajectory("mid_ship_to_1R.right");
          System.out.println("mid_ship_to_1R_right"+ mid_ship_to_1R_right.toString());

          right_ship_to_1R_left = PathfinderFRC.getTrajectory("right_ship_to_1R.left");
          System.out.println("right_ship_to_1R_left"+ right_ship_to_1R_left.toString());
          right_ship_to_1R_right = PathfinderFRC.getTrajectory("right_ship_to_1R.right");
          System.out.println("right_ship_to_1R_right"+ right_ship_to_1R_right.toString());

          backUp_ship_1L_left = PathfinderFRC.getTrajectory("backUp_ship_1L.left");
          System.out.println("backUp_ship_1L_left"+ backUp_ship_1L_left.toString());
          backUp_ship_1L_right = PathfinderFRC.getTrajectory("backUp_ship_1L.right");
          System.out.println("backUp_ship_1L_right"+ backUp_ship_1L_right.toString());

          backUp_ship_1R_left = PathfinderFRC.getTrajectory("backUp_ship_1R.left");
          System.out.println("backUp_ship_1R_left"+ backUp_ship_1R_left.toString());
          backUp_ship_1R_right = PathfinderFRC.getTrajectory("backUp_ship_1R.right");
          System.out.println("backUp_ship_1R_right"+ backUp_ship_1R_right.toString());


           R1_to_lsR_left= PathfinderFRC.getTrajectory("1R_to_lsR.left");
          System.out.println("R1_to_lsR_left"+ backUp_ship_1R_left.toString());
          R1_to_lsR_right = PathfinderFRC.getTrajectory("1R_to_lsR.right");
          System.out.println("R1_to_lsR_right"+ backUp_ship_1R_right.toString());

          backUp_ship_2L_left = PathfinderFRC.getTrajectory("backUp_ship_2L.left");
          System.out.println("backUp_ship_2L_left"+ backUp_ship_2L_left.toString());
          backUp_ship_2L_right = PathfinderFRC.getTrajectory("backUp_ship_2L.right");
          System.out.println("backUp_ship_2L_right"+ backUp_ship_2L_right.toString());

          lsR_to_Right_2R_left = PathfinderFRC.getTrajectory("lsR_to_Right_ship2R.left");
          System.out.println("lsR_to_Right_2R_left"+ lsR_to_Right_2R_left.toString());
          lsR_to_Right_2R_right = PathfinderFRC.getTrajectory("lsR_to_Right_ship2R.right");
          System.out.println("lsR_to_Right_2R_right"+ lsR_to_Right_2R_right.toString());

          backUp_ship_2R_left = PathfinderFRC.getTrajectory("backUp_ship_2R.left");
          System.out.println("backUp_ship_2R_left"+ backUp_ship_2R_left.toString());
          backUp_ship_2R_right = PathfinderFRC.getTrajectory("backUp_ship_2R.right");
          System.out.println("backUp_ship_2R_right"+ backUp_ship_2R_right.toString());

          right_ship_to_2R_left = PathfinderFRC.getTrajectory("right_ship_to_2R.left");
          System.out.println("right_ship_to_2R_left"+ right_ship_to_2R_left.toString());
          right_ship_to_2R_right = PathfinderFRC.getTrajectory("right_ship_to_2R.right");
          System.out.println("right_ship_to_2R_right"+ right_ship_to_2R_right.toString());

          left_ship_to_1L_left = PathfinderFRC.getTrajectory("left_ship_to_1L.left");
          System.out.println("left_ship_to_1L_left"+ left_ship_to_1L_left.toString());
          left_ship_to_1L_right = PathfinderFRC.getTrajectory("left_ship_to_1L.right");
          System.out.println("left_ship_to_1L_right"+ left_ship_to_1L_right.toString());

          exit_level_two_left = PathfinderFRC.getTrajectory("exit_level_two.left");
          System.out.println("exit_level_two_left"+exit_level_two_left.toString());
          exit_level_two_right = PathfinderFRC.getTrajectory("exit_level_two.right");
          System.out.println("exit_level_two_right"+exit_level_two_right.toString());

          testL = PathfinderFRC.getTrajectory("testPath.left");
          System.out.println("testL" + testL.toString());
          testR = PathfinderFRC.getTrajectory("testPath.right");
          System.out.println("testR" + testR.toString());

          // revL = PathfinderFRC.getTrajectory("testPath.left");
          // revR = PathfinderFRC.getTrajectory("testPath.right");

          drive1L = PathfinderFRC.getTrajectory("drive1.left");
          drive1R = PathfinderFRC.getTrajectory("drive1.right");
          drive2R = PathfinderFRC.getTrajectory("HatchGrab.right");
          drive2L = PathfinderFRC.getTrajectory("HatchGrab.left");
          // drive3L = PathfinderFRC.getTrajectory("return.left");
          // drive3R = PathfinderFRC.getTrajectory("return.right");
          // driveFinR = PathfinderFRC.getTrajectory("backUp2.right");
          // driveFinL = PathfinderFRC.getTrajectory("backUp2.left");

          } catch (IOException e) {
               System.err.println("There was a problem loading paths, fix it! " + e.getMessage());
          }
     }
}
