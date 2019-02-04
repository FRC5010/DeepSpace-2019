/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point3;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class Vision extends Subsystem {

  private static NetworkTable table;

  // current values
  private static double tXc = 0.0;
  private static double tYc = 0.0;
  private static double tAc = 0.0;
  private static double tvXc = 0.0;
  private static double tvYc = 0.0;
  private static double tvZc = 0.0;
  private static double tvDistanceC = 0.0;
  private static double rotAngleC = 0.0;
  private static double shiftAngleC = 0.0;
  private static double tDistanceC = 0.0;
  private static double tSkewC = 0.0;
  private static double tShortC = 0.0;
  private static double tLongC = 0.0;
  private static double tHorC = 0.0;
  private static double tVertC = 0.0;
  private static double latencyC = 0.0;
  private static double[] cornXc;
  private static double[] cornYc;

  // smoothed values
  private static double tXs = 0.0;
  private static double tYs = 0.0;
  private static double tAs = 0.0;
  private static double tvDistanceS = 0.0; // Distance from tVec
  private static double rotAngleS = 0.0;
  private static double shiftAngleS = 0.0;
  private static double tDistanceS = 0.0; // Distance from tY
  private static double tSkewS = 0.0;
  private static double tShortS = 0.0;
  private static double tLongS = 0.0;
  private static double tHorS = 0.0;
  private static double tVertS = 0.0;
  private static double latencyS = 0.0;
  private static double[] cornXs;
  private static double[] cornYs;

  private static boolean tValidc = false;
  private static boolean tValids = false;
  private static long lastValid = 0;

  public static final double LIME_LIGHT_HEIGHT = 36;
  public static final double targetHeight = 24;
  private MatOfPoint3f mObjectPoints;
  private Mat mCameraMatrix;
  private MatOfDouble mDistortionCoefficients;

  // general properties
  public static boolean lightOn = true;
  public static int pipelineNumber = 1;

  public Vision() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    mObjectPoints = new MatOfPoint3f(new Point3(0.0, 0.0, 0.0), // bottom right
        new Point3(-1.9363, 0.5008, 0.0), // bottom left
        new Point3(-0.5593, 5.8258, 0.0), // top-left
        new Point3(1.377, 5.325, 0.0) // top-right
    );

    mCameraMatrix = Mat.eye(3, 3, CvType.CV_64F);
    mCameraMatrix.put(0, 0, 2.5751292067328632e+02);
    mCameraMatrix.put(0, 2, 1.5971077914723165e+02);
    mCameraMatrix.put(1, 1, 2.5635071715912881e+02);
    mCameraMatrix.put(1, 2, 1.1971433393615548e+02);

    mDistortionCoefficients = new MatOfDouble(2.9684613693070039e-01, -1.4380252254747885e+00, -2.2098421479494509e-03,
        -3.3894563533907176e-03, 2.5344430354806740e+00);
  }

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
      latencyC = table.getEntry("tl").getDouble(0.0);
      cornXc = table.getEntry("tcornx").getDoubleArray(new double[0]);
      cornYc = table.getEntry("tcorny").getDoubleArray(new double[0]);

      matrixMathOnCorners();

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
        latencyS = latencyC;
        cornXs = cornXc;
        cornYs = cornYc;
        rotAngleS = rotAngleC;
        shiftAngleS = shiftAngleC;
        tvDistanceS = tvDistanceC;
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
        tAs = tYs = tXs = tDistanceS = tSkewS = tShortS = Double.NaN;
        tLongS = tHorS = tVertS = latencyS = Double.NaN;
        rotAngleS = shiftAngleS = tvDistanceS = Double.NaN;
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
    SmartDashboard.putNumber("Target Rotation Angle raw", Double.isNaN(rotAngleC) ? 0.0 : rotAngleC);
    SmartDashboard.putNumber("Target Shift Angle raw ", Double.isNaN(shiftAngleC) ? 0.0 : shiftAngleC);
    SmartDashboard.putNumber("Target Trans Distance raw", Double.isNaN(tvDistanceC) ? 0.0 : tvDistanceC);
    SmartDashboard.putNumber("Limelight Latency raw", Double.isNaN(latencyC) ? 0.0 : latencyC);
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
    SmartDashboard.putNumber("Target Rotation Angle smoothed", Double.isNaN(rotAngleS) ? 0.0 : rotAngleS);
    SmartDashboard.putNumber("Target Shift Angle smoothed ", Double.isNaN(shiftAngleS) ? 0.0 : shiftAngleS);
    SmartDashboard.putNumber("Target Trans Distance smoothed", Double.isNaN(tvDistanceS) ? 0.0 : tvDistanceS);
    SmartDashboard.putNumber("Limelight Latency smoothed", Double.isNaN(latencyS) ? 0.0 : latencyS);
  }

  private void matrixMathOnCorners() {
    PointFinder pointFinder = new PointFinder(cornXs, cornYs);
    // System.out.println(pointFinder);
    MatOfPoint2f imagePoints = new MatOfPoint2f(pointFinder.getBottomRight(), pointFinder.getBottomLeft(),
        pointFinder.getTopLeft(), pointFinder.getTopRight());

    Mat rotationVector = new Mat();
    Mat translationVector = new Mat();
    Calib3d.solvePnP(mObjectPoints, imagePoints, mCameraMatrix, mDistortionCoefficients, rotationVector,
        translationVector);
    SmartDashboard.putNumber("rotVec0", rotationVector.get(0, 0)[0]);
    SmartDashboard.putNumber("rotVec1", rotationVector.get(1, 0)[0]);
    SmartDashboard.putNumber("rotVec2", rotationVector.get(2, 0)[0]);
    tvXc = translationVector.get(0, 0)[0];
    tvYc = translationVector.get(1, 0)[0];
    tvZc = translationVector.get(2, 0)[0];
    SmartDashboard.putNumber("tranVec X", tvXc);
    SmartDashboard.putNumber("tranVec Y", tvYc);
    SmartDashboard.putNumber("tranVec Z", tvZc);

    Mat rotation = new Mat();
    Calib3d.Rodrigues(rotationVector, rotation);
    Mat rotation_inverted = new Mat();
    Core.transpose(rotation, rotation_inverted);
    Mat negTransVect = new Mat();
    negTransVect.put(0, 0, -tvXc);
    negTransVect.put(1, 0, -tvYc);
    negTransVect.put(2, 0, -tvZc);
    Mat pZeroWorld = rotation_inverted.mul(negTransVect);
    double pZeroWorld0 = pZeroWorld.get(0, 0)[0];
    double pZeroWorld2 = pZeroWorld.get(2, 0)[0];

    rotAngleC = Math.atan2(tvXc, tvZc);
    tvDistanceC = Math.sqrt(Math.pow(tvXc, 2) + Math.pow(tvZc, 2));
    shiftAngleC = Math.atan2(pZeroWorld0, pZeroWorld2);
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
      rotAngleS = (previousPose.rotationAngle + rotAngleC) / 2.0;
      shiftAngleS = (previousPose.shiftAngle + shiftAngleC) / 2.0;
      tvDistanceS = (previousPose.transVecDistance + tvDistanceC) / 2.0;
      latencyS = Math.floor(latencyC);
      cornXs = cornXc;
      cornYs = cornYc;
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

  public double getLatency() {
    return latencyS;
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

  public double[] getCornX() {
    return cornXs;
  }

  public double[] getCornY() {
    return cornYs;
  }

  public double getRotAngle() {
    return rotAngleS;
  }

  public double getShiftAngle() {
    return shiftAngleS;
  }

  public double getTVDistance() {
    return tvDistanceS;
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
    pipelineNumber = ppipelineNumber;
    if (ppipelineNumber != -1) {
      table.getEntry("camMode").setNumber(0);
    } else {
      table.getEntry("camMode").setNumber(1);
      return;
    }
    table.getEntry("pipeline").setNumber(ppipelineNumber);
  }
}
