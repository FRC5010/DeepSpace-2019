/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.File;
import java.io.IOException;
import java.util.HashMap;
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
     public static Map<MotionProfiles, Trajectory> leftTrajectories;
     public static Map<MotionProfiles, Trajectory> rightTrajectories;

     public static enum MotionProfiles {
          // Middle Start
          MStoShip1L, MStoShip1R,
          // Left Start Side X2 
          LStoShip2L, backUp_ship_2L, Ship2LtoLP,
          // Left Start Front & Side
          left_ship_to_1L, backUp_ship_1L,
          // Right Start Front & Side
          RStoShip1R, backUp_ship_1R, Ship1RtoRP, lsR_to_Right_ship2R,
          // Right Start Side X2
          right_ship_to_2R, backUp_ship_2R,
          // Test routines
          testPath, exit_level_two
     }

     private static boolean errorLoadingPaths = false;

     public static void init() {
          leftTrajectories = new HashMap<>();
          rightTrajectories = new HashMap<>();

          for(MotionProfiles mp : MotionProfiles.values()) {
               loadTrajectories(mp);
          }
     }

     private static void loadTrajectories(MotionProfiles basePathName) {
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
