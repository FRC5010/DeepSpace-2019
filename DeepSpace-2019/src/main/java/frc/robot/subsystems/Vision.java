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
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Add your docs here.
 */
public class Vision extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static NetworkTable table;
  
  public static final double LIME_LIGHT_HEIGHT = 36;
  private static double tXs;
  private static double tYs;
  private static double tAs;
  private static double tXc;
  private static double tYc;
  private static double tAc;
  private static boolean tValids = false;
  private static boolean tValidc = false;
  private static boolean tValidl = false;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void update() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tValidc = table.getEntry("tv").getDouble(0.0) == 0.0 ? false : true;
    SmartDashboard.putBoolean("Valid Target", tValidc);

    tXc = table.getEntry("tx").getDouble(0.0);
    tXs = tValidc ? (tXs + tXc) / 2 : tXs;
    SmartDashboard.putNumber("Target X", tXs);
    
    tYc = table.getEntry("ty").getDouble(0.0);
    tYs = tValidc ? (tYs + tYc) / 2 : tYs;
    SmartDashboard.putNumber("Target Y", tYs);
    
    tAc = table.getEntry("ta").getDouble(0.0);
    tAs = tValidc ? (tAs + tAc) / 2 : tAs;
    SmartDashboard.putNumber("Target Area", tAs);
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

  public boolean isTargetValid() { return tValidc; }
}
