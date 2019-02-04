/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import org.opencv.core.Point;

/**
 * Add your docs here.
 */
public class PointFinder {
    private final Point bottomLeft, bottomRight, topLeft, topRight;
    private final double leftLength, rightLength;

    public PointFinder(double[] cornX, double[] cornY) {
        if (cornX.length != 4 || cornY.length != 4) {
            System.out.println("[ERROR] Could not find 4 points from image");
            topLeft = new Point(0, 0);
            topRight = new Point(0, 0);
            bottomLeft = new Point(0, 0);
            bottomRight = new Point(0, 0);
            leftLength = 0;
            rightLength = 0;
        } else {
            bottomRight = new Point(cornX[0], cornY[0]);
            bottomLeft = new Point(cornX[1], cornY[1]);
            topLeft = new Point(cornX[2], cornY[2]);
            topRight = new Point(cornX[3], cornY[3]);
            leftLength = Math.sqrt(
                    Math.pow(Math.abs(topLeft.x - bottomLeft.x), 2) + Math.pow(Math.abs(topLeft.y - bottomLeft.y), 2));
            rightLength = Math.sqrt(Math.pow(Math.abs(topRight.x - bottomRight.x), 2)
                    + Math.pow(Math.abs(topRight.y - bottomRight.y), 2));
        }
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

    public double getLeftLength() {
        return leftLength;
    }

    public double getRightLength() {
        return rightLength;
    }

    public String toString() {
        return "TL" + topLeft.toString() + " TR" + topRight + "\n" + "BL" + bottomLeft.toString() + " BR" + bottomRight;
    }
}
