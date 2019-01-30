/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Collection;
import java.util.HashMap;
import java.util.Map;

/**
 * This class contains sets of values that coorespond to various locations on the field
 */
public class FieldMap {
    // TODO: Add an enum for each field target
    public static enum Target {
        OUR_LEFT_ROCKET_NEAR_HATCH,
        OUR_LEFT_ROCKET_CARGO,
        OUR_LEFT_ROCKET_FAR_HATCH,

        OUR_LEFT_SHIP_BAY_1,
        OUR_LEFT_SHIP_BAY_2,

        THEIR_LEFT_ROCKET_NEAR_HATCH,
        THEIR_LEFT_ROCKET_CARGO,
        THEIR_LEFT_ROCKET_FAR_HATCH,

        THEIR_RIGHT_ROCKET_NEAR_HATCH,

        THEIR_LEFT_SHIP_BAY_1,
        THEIR_LEFT_SHIP_BAY_2
    }

    public static class Position {
        public final double x; // Feet across from the top left
        public final double y; // Feet down from the top left
        public final double z; // Height of the target if applicable
        public final double angle; // The angle the robot needs to be facing where east = 0
        public Position(double x, double y, double z, double angle) {
            this.x = x; this.y = y; this.angle = angle;
            this.z = z;
        }
        public double distanceFromPosition(Position pos2) {
            // TODO: Calculate the distance between this position and pos2
            return 0.0;
        }
    }
    private static Map<Target, Position> fieldMap;

    static {
        fieldMap = new HashMap<>();
        // TODO: Need to add position data
        fieldMap.put(Target.OUR_LEFT_ROCKET_NEAR_HATCH, new Position(0, 0, 0, 0));
    }

    public static Position getPosition(Target trg) {
        return fieldMap.get(trg);
    }

    public static Target getNearestTarget(Position pos) {
        // TODO: Use the distanceFromPosition code to find the target
        // nearest the passed in position
        Collection<Position> fieldMapPosistions = fieldMap.values();
        Target nearestTarget = Target.OUR_LEFT_ROCKET_CARGO;
        double lastNearestDistance = Double.NaN;
        for (Position nextPos : fieldMapPosistions) {

        }
        return nearestTarget;
    }
}
