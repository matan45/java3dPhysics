package collisionDetection.narrowPhase.sat;

import collisionDetection.util.CollisionUtil;
import math.Vector3f;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class SAT {

    public static boolean isCollide(SATSupport shape1, SATSupport shape2) {

        Set<Vector3f> allAxis = CollisionUtil.combineAxis(shape1, shape2);

        // Check for separation along each axis
        for (Vector3f axis : allAxis) {
            if (isAxisSeparating(axis, shape1, shape2)) {
                return false; // No collision along this axis
            }
        }

        return true; // No separation along any axis, collision detected
    }

    private static boolean isAxisSeparating(Vector3f axis, SATSupport shape1, SATSupport shape2) {
        // Project the shapes onto the axis
        Interval projection1 = shape1.getInterval(axis);
        Interval projection2 = shape2.getInterval(axis);

        // Check for separation between the intervals
        return projection1.getMax() < projection2.getMin() || projection2.getMax() < projection1.getMin();
    }
}
