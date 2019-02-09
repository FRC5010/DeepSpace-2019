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
    public final double matrixRotationAngle;
    public final double matrixApproachAngle;
    public final double matrixDistance;
    public final double leftRightRatio;
    public final double aspectApproachAngle;

    public final long driveTrainEncoderLeft;
    public final long driveTrainEncoderRight;
    public final double heading;
    public final long elevatorEncoder;

    // public static final Map<Long, Pose> poseMap = new HashMap<Long, Pose>();
    public static final List<Pose> poseList = new ArrayList<Pose>();
    private static Pose currentPose = new Pose();
    private static long poseListLimit = 200;

    public Pose(long timestamp) {
        this.timestamp = timestamp;
        limeLightTx = RobotMap.vision.getX();
        limeLightTy = RobotMap.vision.getY();
        limeLightTa = RobotMap.vision.getA();
        limeLightValid = RobotMap.vision.isTargetValid();
        limeLightDistance = RobotMap.vision.getDistance();
        limeLightSkew = RobotMap.vision.getSkew();
        limeLightShort = RobotMap.vision.getShort();
        limeLightLong = RobotMap.vision.getLong();
        limeLightHorizontal = RobotMap.vision.getHor();
        limeLightVertical = RobotMap.vision.getVert();
        limeLightLatency = RobotMap.vision.getLatency();
        matrixRotationAngle = RobotMap.vision.getMatrixRotationAngle();
        matrixApproachAngle = RobotMap.vision.getMatrixApproachAngle();
        matrixDistance = RobotMap.vision.getMatrixDistance();
        leftRightRatio = RobotMap.vision.getLeftRightRatio();
        aspectApproachAngle = RobotMap.vision.getAspectApproachAngle();
        heading = RobotMap.direction.angle();
        driveTrainEncoderLeft = RobotMap.leftEncoder.getRaw();
        SmartDashboard.putNumber("Lencoder",driveTrainEncoderLeft);
        driveTrainEncoderRight = RobotMap.rightEncoder.getRaw();
        SmartDashboard.putNumber("Rencoder",driveTrainEncoderRight);
        elevatorEncoder = 0; // Needs to be updated
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
        matrixRotationAngle = 0.0;
        matrixDistance = 0.0;
        matrixApproachAngle = 0.0;
        leftRightRatio = 0.0;
        aspectApproachAngle = 0.0;

        driveTrainEncoderLeft = 0;
        driveTrainEncoderRight = 0;
        heading = 0.0;
        elevatorEncoder = 0;
    }

    public static Pose getCurrentPose() {
        return currentPose;
    }

    public static Pose update() {
        RobotMap.vision.update();

        currentPose = new Pose(System.currentTimeMillis() / 10); // round down to minimum resolution);
        poseList.add(currentPose);

        if (poseList.size() > poseListLimit) {
            Pose removePose = poseList.remove(0);
            // poseMap.remove(removePose.timestamp, removePose);
        }

        // poseMap.put(currentPose.timestamp, currentPose);
        return currentPose;
    }
}
