/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Add your docs here.
 */
public class Vision extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static NetworkTable table;
  
  //current values
  private static double tXc = 0.0;
  private static double tYc = 0.0;
  private static double tAc = 0.0;
  private static double tDistanceC = 0.0;
  
  //smoothed values
  private static double tXs = 0.0;
  private static double tYs = 0.0;
  private static double tAs = 0.0;
  private static double tDistanceS = 0.0;

  private static boolean tValidc = false;
  private static boolean tValids = false;
  private static long lastValid = 0;

  public static final double LIME_LIGHT_HEIGHT = 36;
  public static final double targetHeight = 24;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void update() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tValidc = table.getEntry("tv").getDouble(0.0) == 0.0 ? false : true;

    if (tValidc) {
      if (!tValids) {
        // If tValids was false, our previous saved position data is also bad (we set to NaN), reset to 0.0.
        //tAs = tYs = tXs = 0.0;
        // Anytime the current frame is valid, the saved valid becomes true
        tValids = true;
      }
      lastValid = System.currentTimeMillis();
    } else { 
      // If target is invalid for more that 0.5 seconds, it is likely off screen
      if (tValids && (lastValid + 500) > System.currentTimeMillis()) {
        // Don't need to run this code if tValids is already false
        tValids = false;
        // If the target has been invalid too long, set to NaN
        tAc = tYc = tXc = Double.NaN;
      }
    }
    
    SmartDashboard.putBoolean("Valid Target c", tValidc);
    SmartDashboard.putBoolean("Valid Target s", tValids);

    if (tValidc && tValids) {
      //get the raw values from the camera, then smooth them out
      tXc = table.getEntry("tx").getDouble(0.0);
      tYc = table.getEntry("ty").getDouble(0.0);
      tAc = table.getEntry("ta").getDouble(0.0);
      tDistanceC = calculateDistance();
      smoothValues();
    } else if (tValids) {
      // We don't currently have valid data, but
      // we can use a projection algorithm
      // Currently, that means not changing the saved values
      // Eventually, we should try to use past pose
      // data to predict current updated values.
    }

    SmartDashboard.putNumber("Target X raw", Double.isNaN(tXc) ? 0.0 : tXc);  
    SmartDashboard.putNumber("Target Y raw", Double.isNaN(tYc) ? 0.0 : tYc);
    SmartDashboard.putNumber("Target Area raw", Double.isNaN(tAc) ? 0.0 : tAc);
    SmartDashboard.putNumber("Target Distance raw", Double.isNaN(tDistanceC) ? 0.0 : tDistanceC);

    SmartDashboard.putNumber("Target X smoothed", Double.isNaN(tXs) ? 0.0 : tXs);  
    SmartDashboard.putNumber("Target Y smoothed", Double.isNaN(tYs) ? 0.0 : tYs);
    SmartDashboard.putNumber("Target Area smoothed", Double.isNaN(tAs) ? 0.0 : tAs);
    SmartDashboard.putNumber("Target Distance smoothed", Double.isNaN(tDistanceS) ? 0.0 : tDistanceS);
  }

  private void smoothValues() {
    Pose previousPose = Pose.poseList.get(Pose.poseList.indexOf(Pose.getCurrentPose()) - 1);
    if (previousPose.limeLightValid && tValids) {
      tXs = (previousPose.limeLightTx + tXc) / 2.0;
      tYs = (previousPose.limeLightTy + tYc) / 2.0;
      tAs = (previousPose.limeLightTa + tAc) / 2.0;
      tDistanceS = (previousPose.limeLightDistance + tDistanceC) / 2.0;
    }
  }

  public double getX() {
    return tXs;
  }
  
  public double getY() {
    return tYs;
  }

  public double getA() {
    return tAs;
  }

  public boolean isTargetValid() {
    return tValids;
  }

  private double calculateDistance() {
    double distance = Double.NaN;
    if (tValids) {
      double yInRadians = Math.toRadians(tYc);
      distance = Math.abs((LIME_LIGHT_HEIGHT - targetHeight) / Math.tan(yInRadians));
    } else {
      distance = Double.NaN;
    }
    return distance;
  }

  public double getDistance() {
    return tDistanceS;
  }
}
