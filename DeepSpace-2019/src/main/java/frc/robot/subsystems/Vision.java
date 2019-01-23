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
  private static Double tXs = Double.valueOf(0);
  private static Double tYs = Double.valueOf(0);
  private static Double tAs = Double.valueOf(0);
  private static Double tXc = Double.valueOf(0);
  private static Double tYc = Double.valueOf(0);
  private static Double tAc = Double.valueOf(0);
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
      tValids = true;
      lastValid = System.currentTimeMillis();
    } else { 
      // If target is invalid for more that 0.5 seconds, it is likely off screen
      if (lastValid + 500 > System.currentTimeMillis()) {
        tValids = false;
      }
    }
    
    //System.out.println("validC: " + tValidc + ", validS: " + tValids);


    tXc = Double.valueOf(table.getEntry("tx").getDouble(0.0));
    Double tXtmp = tValids ? tXs : Double.NaN;
    if (tValidc & tValids) {
      tXs = Double.valueOf(Double.valueOf((tXs + tXc)) / 2.0);
     } else {
       tXs = tXtmp;
     }
    SmartDashboard.putNumber("Target X", (Double.NaN != tXs) ? tXs : 0.0);
    
    tYc = Double.valueOf(table.getEntry("ty").getDouble(0.0));
    Double tYtmp = tValids ? tYs : Double.NaN;
    if (tValidc & tValids) {
      tYs = (tYs + tYc) / 2.0;
     } else {
       tYs = tYtmp;
     }
    SmartDashboard.putNumber("Target Y", (Double.NaN != tYs) ? tYs : 0.0);
    
    tAc = Double.valueOf(table.getEntry("ta").getDouble(0.0));
    Double tAtmp = tValids ? tAs : Double.NaN;
    if (tValidc & tValids) {
      tAs = (tAs + tAc) / 2.0;
     } else {
       tAs = tAtmp;
     }
    SmartDashboard.putNumber("Target Area", (Double.NaN != tAs) ? tAs : 0.0);
  }

  public Double getX() {
    return tXs;
  }
  
  public Double getY() {
    return tYs;
  }

  public Double getA() {
    return tAs;
  }

  public boolean isTargetValid() { return tValids; }
}
