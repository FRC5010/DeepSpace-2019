/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import frc.robot.RobotMap;
import frc.robot.subsystems.Vision.Values;

/**
 * Add your docs here.
 */
public class Pose {
    public final long timestamp; // round down to minimum resolution
    public final Vision.Values limeLight;

    public final long driveTrainEncoderLeft;
    public final long driveTrainEncoderRight;
    public final double heading;
    public final double elevatorEncoder;

    // public static final Map<Long, Pose> poseMap = new HashMap<Long, Pose>();
    public static final List<Pose> poseList = new ArrayList<Pose>();
    private static Pose currentPose = new Pose();
    private static long poseListLimit = 200;

    public Pose(long timestamp) {
        this.timestamp = timestamp;
        limeLight = new Values(0.0);
        limeLight.tX = RobotMap.vision.getX();
        limeLight.tY = RobotMap.vision.getY();
        limeLight.tA = RobotMap.vision.getA();
        limeLight.tValid = RobotMap.vision.isTargetValid();
        limeLight.tDistance = RobotMap.vision.getDistance();
        limeLight.tSkew = RobotMap.vision.getSkew();
        limeLight.tShort = RobotMap.vision.getShort();
        limeLight.tLong = RobotMap.vision.getLong();
        limeLight.tHor = RobotMap.vision.getHor();
        limeLight.tVert = RobotMap.vision.getVert();
        limeLight.latency = RobotMap.vision.getLatency();
        limeLight.matrixRotationAngle = RobotMap.vision.getMatrixRotationAngle();
        limeLight.matrixApproachAngle = RobotMap.vision.getMatrixApproachAngle();
        limeLight.matrixDistance = RobotMap.vision.getMatrixDistance();
        limeLight.leftRightRatio = RobotMap.vision.getLeftRightRatio();
        limeLight.aspectApproachAngle = RobotMap.vision.getAspectApproachAngle();
        heading = RobotMap.direction.angle();
        driveTrainEncoderLeft = RobotMap.distance.getLeftRaw();
        driveTrainEncoderRight = RobotMap.distance.getRightRaw();
        elevatorEncoder = RobotMap.elevator.getCurrentPosition();
    }

    private Pose() {
        timestamp = 0;
        limeLight = new Values(0.0);
        driveTrainEncoderLeft = 0;
        driveTrainEncoderRight = 0;
        heading = 0.0;
        elevatorEncoder = 0.0;
    }

    public static Pose getCurrentPose() {
        return currentPose;
    }

    /** Might return less than the requested size */
    public static List<Pose> getPreviousPoses(int setSize) {
        if (poseList.size() <= setSize) {
            setSize = poseList.size() - 1;
        }
        return poseList.subList(poseList.size() - setSize - 1, poseList.size() - 1);
    }

    public static Pose update(long timestamp) {
        RobotMap.vision.update(timestamp);
        
        currentPose = new Pose(timestamp); // round down to minimum resolution);
        poseList.add(currentPose);

        if (poseList.size() > poseListLimit) {
            Pose removePose = poseList.remove(0);
            // poseMap.remove(removePose.timestamp, removePose);
        }

        // poseMap.put(currentPose.timestamp, currentPose);
        return currentPose;
    }
}
