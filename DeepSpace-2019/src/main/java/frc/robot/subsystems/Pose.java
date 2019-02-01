/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Pose {
    public final long timestamp; // round down to minimum resolution
    public final double limeLightTx;
    public final double limeLightTy;
    public final double limeLightTa;
    public final boolean limeLightValid;
    public final double limeLightDistance;
    public final double limeLightSkew;
    public final double limeLightShort;
    public final double limeLightLong;
    public final double limeLightHorizontal;
    public final double limeLightVertical;
    public final double limeLightLatency;

    public final double driveTrainEncoderLeft;
    public final double driveTrainEncoderRight;
    public final double heading;
    public final double elevatorEncoder;

    // public static final Map<Long, Pose> poseMap = new HashMap<Long, Pose>();
    public static final List<Pose> poseList = new ArrayList<Pose>();
    private static Pose currentPose = new Pose();
    private static long poseListLimit = 200;

    public Pose(double tx, double ty, double ta, boolean tv, double distance,
                double tSkew, double tShort, double tLong, double tHorizontal,
                double tVertical, double tLatency, 
                double el, double er, double h, double ee) {
        timestamp = System.currentTimeMillis() / 10; // round down to minimum resolution
        limeLightTx = tx;
        limeLightTy = ty;
        limeLightTa = ta;
        limeLightDistance = distance;
        limeLightValid = tv;
        limeLightSkew = tSkew;
        limeLightShort = tShort;
        limeLightLong = tLong;
        limeLightHorizontal = tHorizontal;
        limeLightVertical = tVertical;
        limeLightLatency = tLatency;
        driveTrainEncoderLeft = el;
        driveTrainEncoderRight = er;
        heading = h;
        elevatorEncoder = ee;
    }

    private Pose() {
        timestamp = 0;
        limeLightTx = 0.0;
        limeLightTy = 0.0;
        limeLightTa = 0.0;
        limeLightDistance = 0.0;
        limeLightValid = false;
        limeLightSkew = 0.0;
        limeLightShort = 0.0;
        limeLightLong = 0.0;
        limeLightHorizontal = 0.0;
        limeLightVertical = 0.0;
        limeLightLatency = 0.0;
        driveTrainEncoderLeft = 0.0;
        driveTrainEncoderRight = 0.0;
        heading = 0.0;
        elevatorEncoder = 0.0;
    }
    
    public static Pose getCurrentPose() {
        return currentPose;
    }

    public static Pose update() {
        RobotMap.vision.update();

        double limeLightTx = RobotMap.vision.getX();
        double limeLightTy = RobotMap.vision.getY();
        double limeLightTa = RobotMap.vision.getA();
        boolean limeLightValid = RobotMap.vision.isTargetValid();
        double limeLightDistance = RobotMap.vision.getDistance();
        double limeLightSkew = RobotMap.vision.getSkew();
        double limeLightShort = RobotMap.vision.getShort();
        double limeLightLong = RobotMap.vision.getLong();
        double limeLightHorizontal = RobotMap.vision.getHor();
        double limeLightVertical = RobotMap.vision.getVert();
        // TODO: Add all of the limeLight fields
        // TODO: Add heading, encoder output, etc...
        
        currentPose = new Pose(limeLightTx, limeLightTy, limeLightTa, 
                               limeLightValid, limeLightDistance, limeLightSkew,
                               limeLightShort, limeLightLong, limeLightHorizontal, limeLightVertical,
                               0.0, 0.0, 0.0, 0.0, 0.0);
        poseList.add(currentPose);

        if (poseList.size() > poseListLimit) {
            Pose removePose = poseList.remove(0);
            // poseMap.remove(removePose.timestamp, removePose);
        }

        // poseMap.put(currentPose.timestamp, currentPose);
        return currentPose;
    }
}
