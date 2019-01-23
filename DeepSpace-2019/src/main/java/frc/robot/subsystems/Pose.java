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
    public final long timestamp;
    public final Double limeLightTx;
    public final Double limeLightTy;
    public final Double limeLightTa;
    public final Double limeLightTl;
    public final Double limeLightTs;
    public final boolean limeLightValid;

    public final double driveTrainEncoderLeft;
    public final double driveTrainEncoderRight;
    public final double heading;
    public final double elevatorEncoder;

    //public static final Map<Long, Pose> poseMap = new HashMap<Long, Pose>();
    public static final List<Pose> poseList = new ArrayList<Pose>();
    private static Pose currentPose;
    private static long poseListLimit = 200;

    public Pose(Double tx, Double ty, Double ta, boolean tv, Double tl, Double ts, double el, double er, double h, double ee) {
        timestamp = System.currentTimeMillis() / 10; // round down to minimum resolution
        limeLightTa = ta;
        limeLightTl = tl;
        limeLightTs = ts;
        limeLightTx = tx;
        limeLightTy = ty;
        limeLightValid = tv;
        driveTrainEncoderLeft = el;
        driveTrainEncoderRight = er;
        heading = h;
        elevatorEncoder = ee;
    }

    public static Pose currentPose() { return currentPose; }

    public static Pose update() {
        RobotMap.vision.update();
     
        Double limeLightTa = RobotMap.vision.getA();
        Double limeLightTx = RobotMap.vision.getX();
        Double limeLightTy = RobotMap.vision.getY();
        boolean limeLightValid = RobotMap.vision.isTargetValid();

        currentPose = new Pose(limeLightTx, limeLightTy, limeLightTa, limeLightValid, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        
        poseList.add(currentPose);
        if (poseList.size() > poseListLimit) {
            Pose removePose = poseList.remove(0);
            //poseMap.remove(removePose.timestamp, removePose);
        }

        //poseMap.put(currentPose.timestamp, currentPose);

        // TODO: Remove, only being called here so it always logs
        VisionAssistedDrive.distanceFromTarget();
        return currentPose;
    }
}
