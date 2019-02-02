/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;

/**
 * Add your docs here.
 */
public class PointFinder {
    private final Point bottomLeft, bottomRight, topLeft, topRight;

    public PointFinder(double[] cornX, double[] cornY) {
        topLeft = new Point(cornX[0], cornY[0]);
        topRight = new Point(cornX[1], cornY[1]);
        bottomLeft = new Point(cornX[2], cornY[2]);
        bottomRight = new Point(cornX[3], cornY[3]);
    }

    public Point getBottomRight() {
        return bottomRight;
    }

    public Point getBottomLeft() {
        return bottomLeft;
    }

    public Point getTopRight() {
        return topRight;
    }

    public Point getTopLeft() {
        return topLeft;
    }
}
