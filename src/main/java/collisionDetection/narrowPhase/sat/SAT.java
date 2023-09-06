package collisionDetection.narrowPhase.sat;

import math.Vector3f;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class SAT {

    public static boolean isCollide(SATSupport shape1, SATSupport shape2) {
        Set<Vector3f> allAxis = new HashSet<>();
        List<Vector3f> shape1Axis = shape1.getAxis();
        List<Vector3f> shape2Axis = shape2.getAxis();

        for (Vector3f axis1 : shape1Axis) {
            for (Vector3f axis2 : shape2Axis) {
                allAxis.add(axis1.cross(axis2).normalize());
            }
        }
        allAxis.addAll(shape1Axis);
        allAxis.addAll(shape2Axis);

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
