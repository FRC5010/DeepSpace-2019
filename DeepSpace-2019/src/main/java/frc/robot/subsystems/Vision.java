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
  private static double tgtXsave = 0.0;
  private static double tgtYsave = 0.0;
  private static double tgtAsave = 0.0;
  private static double tgtXcur = 0.0;
  private static double tgtYcur = 0.0;
  private static double tgtAcur = 0.0;
  private static boolean tgtValidCur = false;
  private static boolean tgtValidSave = false;
  private static long lastValid = 0;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void update() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tgtValidCur = table.getEntry("tv").getDouble(0.0) == 0.0 ? false : true;
    SmartDashboard.putBoolean("Valid Target", tgtValidCur);

    if (tgtValidCur) {
      if (!tgtValidSave) {
        // If tgtValidSave was false, our previous saved position data
        // is also bad (we set to NaN), reset to 0.0.
        tgtAsave = tgtYsave = tgtXsave = 0.0;
        // Anytime the current frame is valid, the saved valid becomes true
        tgtValidSave = true;
      }
      lastValid = System.currentTimeMillis();
    } else { 
      // If target is invalid for more that 0.5 seconds, it is likely off screen
      if (tgtValidSave && (lastValid + 500) > System.currentTimeMillis()) {
        // Don't need to run this code if tgtValidSave is already false
        tgtValidSave = false;
        // If the target has been invalid too long, set to NaN
        tgtAsave = tgtYsave = tgtXsave = Double.NaN;
      }
    }
    
    if (tgtValidCur) {
      tgtXcur = table.getEntry("tx").getDouble(0.0);
      tgtYcur = table.getEntry("ty").getDouble(0.0);
      tgtAcur = table.getEntry("ta").getDouble(0.0);
      // Use update algorithm
      // Right not this is just averaging the last saved
      // data with the current data, but this is not
      // enough to really smooth out the data and needs
      // to rely on more pose data to better smooth things out
      tgtXsave = (tgtXsave + tgtXcur) / 2.0;
      tgtYsave = (tgtYsave + tgtYcur) / 2.0;
      tgtAsave = (tgtAsave + tgtAcur) / 2.0;
    } else if (tgtValidSave) {
      // We don't currently have valid data, but
      // we can use a projection algorithm
      // Currently, that means not changing the saved values
      // Eventually, we should try to use past pose
      // data to predict current updated values.
    }
    SmartDashboard.putNumber("Target X", (Double.isNaN(tgtXsave) ? 0.0 : tgtXsave));  
    SmartDashboard.putNumber("Target Y", (Double.isNaN(tgtYsave) ? 0.0 : tgtYsave ));
    SmartDashboard.putNumber("Target Area", (Double.isNaN(tgtAsave) ? 0.0 : tgtAsave));
  }

  public double getX() {
    return tgtXsave;
  }
  
  public double getY() {
    return tgtYsave;
  }

  public double getA() {
    return tgtAsave;
  }

  public boolean isTargetValid() { return tgtValidSave; }
}
