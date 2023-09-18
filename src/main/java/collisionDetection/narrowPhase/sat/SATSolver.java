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

        float A = projection1.getMin();
        float B = projection1.getMax();
        float C = projection2.getMin();
        float D = projection2.getMax();

        if (A <= C && B >= C)
            return Math.abs(C - B);
        return Math.abs(A - D);
    }

    private List<Vector3f> calculateContactPoints(SATSupport shape1, SATSupport shape2, Vector3f normal, float depth) {
        List<Vector3f> contacts = new ArrayList<>();

        // Calculate the contact points using the collision normal and depth
        Vector3f contactPoint1 = new Vector3f(normal).mul(depth / 2);
        Vector3f contactPoint2 = new Vector3f(normal).mul(-depth / 2);

        // Translate the contact points to the center of the shapes
        Vector3f center1 = calculateCenter(shape1);
        Vector3f center2 = calculateCenter(shape2);

        contacts.add(contactPoint1.add(center1));
        contacts.add(contactPoint2.add(center2));

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

    private void calculateProjection(SATSupport shape, Vector3f axis, Vector3f max, Vector3f min) {
        List<Vector3f> vertices = shape.getVertices();

        float minProjection = Float.MAX_VALUE;
        float maxProjection = -Float.MAX_VALUE;

        for (Vector3f vertex : vertices) {
            // Project each vertex onto the axis and update min and max projections
            float projection = vertex.dot(axis);
            if (projection < minProjection) {
                min.set(vertex);
            }
            if (projection > maxProjection) {
                max.set(vertex);
            }
        }
    }

}
