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
  private static Double tXs;
  private static Double tYs;
  private static Double tAs;
  private static Double tXc;
  private static Double tYc;
  private static Double tAc;
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
    tXc = table.getEntry("tx").getDouble(0.0);
    tXs = tValidc & tValids ? (tXs + tXc) / 2 : tValids ? tXs : null;
    SmartDashboard.putNumber("Target X", tXs);
    
    tYc = table.getEntry("ty").getDouble(0.0);
    tYs = tValidc & tValids ? (tYs + tYc) / 2 : tValids ? tYs : null;
    SmartDashboard.putNumber("Target Y", tYs);
    
    tAc = table.getEntry("ta").getDouble(0.0);
    tAs = tValidc & tValids ? (tAs + tAc) / 2 : tValids ? tAs : null;
    SmartDashboard.putNumber("Target Area", tAs);
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
