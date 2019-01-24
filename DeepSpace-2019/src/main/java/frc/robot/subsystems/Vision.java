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
  
  public static final double LIME_LIGHT_HEIGHT = 36;
  //current values
  private static double tXc = 0.0;
  private static double tYc = 0.0;
  private static double tAc = 0.0;
  //saved values
  private static double tXs = 0.0;
  private static double tYs = 0.0;
  private static double tAs = 0.0;

  private static boolean tValidc = false;
  private static boolean tValids = false;
  private static long lastValid = 0;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void update() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tValidc = table.getEntry("tv").getDouble(0.0) == 0.0 ? false : true;
    SmartDashboard.putBoolean("Valid Target", tValidc);

    if (tValidc) {
      if (!tValids) {
        // If tValids was false, our previous saved position data is also bad (we set to NaN), reset to 0.0.
        tAs = tYs = tXs = 0.0;
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
        tAs = tYs = tXs = Double.NaN;
      }
    }
    
    if (tValidc && tValids) {
      tXc = table.getEntry("tx").getDouble(0.0);
      tYc = table.getEntry("ty").getDouble(0.0);
      tAc = table.getEntry("ta").getDouble(0.0);
      // Use update algorithm
      // Right now this is just averaging the last saved
      // data with the current data, but this is not
      // enough to really smooth out the data and needs
      // to rely on more pose data to better smooth things out
      tXs = (tXs + tXc) / 2.0;
      tYs = (tYs + tYc) / 2.0;
      tAs = (tAs + tAc) / 2.0;
    } else if (tValids) {
      // We don't currently have valid data, but
      // we can use a projection algorithm
      // Currently, that means not changing the saved values
      // Eventually, we should try to use past pose
      // data to predict current updated values.
    }

    SmartDashboard.putNumber("Target X", (Double.isNaN(tXs) ? 0.0 : tXs));  
    SmartDashboard.putNumber("Target Y", (Double.isNaN(tYs) ? 0.0 : tYs ));
    SmartDashboard.putNumber("Target Area", (Double.isNaN(tAs) ? 0.0 : tAs));

  /*
    //actually getting the values
    if (tValid) {
      tx = table.getEntry("tx").getDouble(0.0);
      ty = table.getEntry("ty").getDouble(0.0);
      ta = table.getEntry("ta").getDouble(0.0);
    } else {
      tx = ty = ta = Double.NaN;
    }
    
  */
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
}
