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
  
  private static NetworkTable table;

  // current values
  private static double tXc = 0.0;
  private static double tYc = 0.0;
  private static double tAc = 0.0;
  private static double tDistanceC = 0.0;
  private static double tSkewC = 0.0;
  private static double tShortC = 0.0;
  private static double tLongC = 0.0;
  private static double tHorC = 0.0;
  private static double tVertC = 0.0;

  // smoothed values
  private static double tXs = 0.0;
  private static double tYs = 0.0;
  private static double tAs = 0.0;
  private static double tDistanceS = 0.0;
  private static double tSkewS = 0.0;
  private static double tShortS = 0.0;
  private static double tLongS = 0.0;
  private static double tHorS = 0.0;
  private static double tVertS = 0.0;

  private static boolean tValidc = false;
  private static boolean tValids = false;
  private static long lastValid = 0;

  public static final double LIME_LIGHT_HEIGHT = 36;
  public static final double targetHeight = 24;

  // general properties
  public static boolean lightOn = true;
  public static int pipelineNumber = 1;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void update() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tValidc = table.getEntry("tv").getDouble(0.0) == 0.0 ? false : true;

    if (tValidc) {
      // get the raw values from the camera
      tXc = table.getEntry("tx").getDouble(0.0);
      tYc = table.getEntry("ty").getDouble(0.0);
      tAc = table.getEntry("ta").getDouble(0.0);
      tDistanceC = calculateDistance();
      tSkewC = table.getEntry("ts").getDouble(0.0);
      tShortC = table.getEntry("tshort").getDouble(0.0);
      tLongC = table.getEntry("tlong").getDouble(0.0);
      tHorC = table.getEntry("thor").getDouble(0.0);
      tVertC = table.getEntry("tvert").getDouble(0.0);

      if (!tValids) {
        // If tValids was false, our previous saved position data is also bad (we set to
        // NaN), reset to our first raw values.
        tXs = tXc;
        tYs = tYc;
        tAs = tAc;
        tDistanceS = tDistanceC;
        tSkewS = tSkewC;
        tShortS = tShortC;
        tLongS = tLongC;
        tHorS = tHorC;
        tVertS = tVertC;
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
        tAs = tYs = tXs = tDistanceS = tSkewS = tShortS = tLongS = tHorS = tVertS = Double.NaN;
      }
    }

    if (tValidc) {
      smoothValues();
    } else if (tValids) {
      // We don't currently have valid data, but
      // we can use a projection algorithm
      // Currently, that means not changing the saved values
      // Eventually, we should try to use past pose
      // data to predict current updated values.
    }

    
    SmartDashboard.putBoolean("Valid Target c", tValidc);
    SmartDashboard.putBoolean("Valid Target s", tValids);

    // outputting all current/raw values
    SmartDashboard.putNumber("Target X raw", Double.isNaN(tXc) ? 0.0 : tXc);
    SmartDashboard.putNumber("Target Y raw", Double.isNaN(tYc) ? 0.0 : tYc);
    SmartDashboard.putNumber("Target Area raw", Double.isNaN(tAc) ? 0.0 : tAc);
    SmartDashboard.putNumber("Target Distance raw", Double.isNaN(tDistanceC) ? 0.0 : tDistanceC);
    SmartDashboard.putNumber("Target Skew raw", Double.isNaN(tSkewC) ? 0.0 : tSkewC);
    SmartDashboard.putNumber("Target Short raw", Double.isNaN(tShortC) ? 0.0 : tShortC);
    SmartDashboard.putNumber("Target Long raw", Double.isNaN(tLongC) ? 0.0 : tLongC);
    SmartDashboard.putNumber("Target Horizontal raw", Double.isNaN(tHorC) ? 0.0 : tHorC);
    SmartDashboard.putNumber("Target Vertical raw", Double.isNaN(tVertC) ? 0.0 : tVertC);

    // outputing all smoothed values
    SmartDashboard.putNumber("Target X smoothed", Double.isNaN(tXs) ? 0.0 : tXs);
    SmartDashboard.putNumber("Target Y smoothed", Double.isNaN(tYs) ? 0.0 : tYs);
    SmartDashboard.putNumber("Target Area smoothed", Double.isNaN(tAs) ? 0.0 : tAs);
    SmartDashboard.putNumber("Target Distance smoothed", Double.isNaN(tDistanceS) ? 0.0 : tDistanceS);
    SmartDashboard.putNumber("Target Skew smoothed", Double.isNaN(tSkewS) ? 0.0 : tSkewS);
    SmartDashboard.putNumber("Target Short smoothed", Double.isNaN(tShortS) ? 0.0 : tShortS);
    SmartDashboard.putNumber("Target Long smoothed", Double.isNaN(tLongS) ? 0.0 : tLongS);
    SmartDashboard.putNumber("Target Horizontal smoothed", Double.isNaN(tHorS) ? 0.0 : tHorS);
    SmartDashboard.putNumber("Target Vertical smoothed", Double.isNaN(tVertS) ? 0.0 : tVertS);
  }

  private void smoothValues() {
    Pose previousPose = Pose.getCurrentPose();
    if (previousPose.limeLightValid && tValids) {
      tXs = (previousPose.limeLightTx + tXc) / 2.0;
      tYs = (previousPose.limeLightTy + tYc) / 2.0;
      tAs = (previousPose.limeLightTa + tAc) / 2.0;
      tDistanceS = (previousPose.limeLightDistance + tDistanceC) / 2.0;
      tSkewS = (previousPose.limeLightSkew + tSkewC) / 2.0;
      tShortS = (previousPose.limeLightShort + tShortC) / 2.0;
      tLongS = (previousPose.limeLightLong + tLongC) / 2.0;
      tHorS = (previousPose.limeLightHorizontal + tHorC) / 2.0;
      tVertS = (previousPose.limeLightVertical + tVertC) / 2.0;
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
    if (tValidc) {
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

  public double getSkew() {
    return tSkewS;
  }

  public double getShort() {
    return tShortS;
  }

  public double getLong() {
    return tLongS;
  }

  public double getHor() {
    return tHorS;
  }

  public double getVert() {
    return tVertS;
  }

  public void toggleLimelight() {
    lightOn = !lightOn;
    table.getEntry("ledMode").setNumber(lightOn ? 3 : 1);
  }

  public void toggleLimelight(boolean forceOn) {
    lightOn = forceOn;
    table.getEntry("ledMode").setNumber(forceOn ? 3 : 1);
  }

  // if pipelineNumber = -1, switch to driver's camera
  public void changePipeline(int ppipelineNumber) {
  //   pipelineNumber = ppipelineNumber;
  //   if (ppipelineNumber != -1) {
  //     table.getEntry("camMode").setNumber(0);
  //   } else {
  //     table.getEntry("camMode").setNumber(1);
  //     return;
  //   }
  //   table.getEntry("pipeline").setNumber(ppipelineNumber);
   }
}
