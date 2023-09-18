package collisionDetection.narrowPhase.sat;

import collisionDetection.narrowPhase.collisionResult.CollisionResult;
import math.Vector3f;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

public class SATSolver {

    public CollisionResult satCollisionResult(SATSupport shape1, SATSupport shape2, Set<Vector3f> allAxis) {

        // Calculate collision normal and penetration depth
        Vector3f normal = calculateCollisionNormal(shape1, shape2, allAxis);
        float depth = calculatePenetrationDepth(shape1, shape2, normal);

        // Calculate and add contact points
        List<Vector3f> contacts = calculateContactPoints(shape1, shape2, normal, depth);

        // Create and return a CollisionResult
        return new CollisionResult(true, normal, depth, contacts);

    }

    public Vector3f calculateCollisionNormal(SATSupport shape1, SATSupport shape2, Set<Vector3f> allAxis) {
        Vector3f smallestOverlapAxis = new Vector3f();
        float smallestOverlap = Float.MAX_VALUE;

        for (Vector3f axis : allAxis) {
            float overlap = calculateOverlapOnAxis(shape1, shape2, axis);

            if (overlap < smallestOverlap) {
                smallestOverlap = overlap;
                smallestOverlapAxis = axis;
            }
        }


        return smallestOverlapAxis.normalize();
    }

    private float calculatePenetrationDepth(SATSupport shape1, SATSupport shape2, Vector3f normal) {
        // Calculate the projection of shapes onto the collision normal
        Interval projection1 = shape1.getInterval(normal);
        Interval projection2 = shape2.getInterval(normal);

        return finedMinProjection(projection1, projection2);
    }

    private float finedMinProjection(Interval projection1, Interval projection2) {
        // Get the min and max values for both projections
        float min1 = projection1.getMin();
        float max1 = projection1.getMax();
        float min2 = projection2.getMin();
        float max2 = projection2.getMax();

        // Initialize an array to store all possible projection combinations
        float[] combinations = new float[4];

        // Calculate the four possible combinations
        combinations[0] = max1 - min2; // projection1 max to projection2 min
        combinations[1] = max1 - max2; // projection1 max to projection2 max
        combinations[2] = min1 - min2; // projection1 min to projection2 min
        combinations[3] = min1 - max2; // projection1 min to projection2 max

        // Find the minimum value among the combinations
        float minOverlap = combinations[0];
        for (int i = 1; i < 4; i++) {
            if (combinations[i] < minOverlap) {
                minOverlap = combinations[i];
            }
        }

        return Math.abs(minOverlap);
    }

    private List<Vector3f> calculateContactPoints(SATSupport shape1, SATSupport shape2, Vector3f normal, float depth) {
        List<Vector3f> contacts = new ArrayList<>();

        // Calculate the contact points using the collision normal and depth
        Vector3f contactPoint1 = new Vector3f(normal).mul(depth / 2);
        Vector3f contactPoint2 = new Vector3f(normal).mul(-depth / 2);

        // Translate the contact points to the center of the shapes
        Vector3f center1 = calculateCenter(shape1);
        Vector3f center2 = calculateCenter(shape2);

        contactPoint1.add(center1);
        contactPoint2.add(center2);

        contacts.add(contactPoint1);
        contacts.add(contactPoint2);

        return contacts;
    }

    private Vector3f calculateCenter(SATSupport shape) {
        List<Vector3f> vertices = shape.getVertices();
        Vector3f center = new Vector3f();
        for (Vector3f vertex : vertices) {
            center.add(vertex);
        }
        center.div(vertices.size());
        return center;
    }

    public float calculateOverlapOnAxis(SATSupport shape1, SATSupport shape2, Vector3f axis) {
        Interval interval1 = shape1.getInterval(axis);
        Interval interval2 = shape2.getInterval(axis);

        // Calculate the overlap between the projections.
        return Math.min(interval1.getMax(), interval2.getMax()) - Math.max(interval1.getMin(), interval2.getMin());
    }

}
