package collisionDetection.narrowPhase.sat;

import collisionDetection.narrowPhase.collision_result.CollisionResult;
import collisionDetection.narrowPhase.gjk.GJKSupport;
import math.Vector3f;

import java.util.List;

public class SATCollisionResolution {
/*
    public CollisionResult SATCollisionResult(SATSupport shape1, SATSupport shape2,boolean isCollide) {

        if (colliding) {
            // Calculate collision normal and penetration depth
            Vector3f normal = calculateCollisionNormal(objectA, objectB);
            float depth = calculatePenetrationDepth(objectA, objectB, normal);

            // Calculate and add contact points
            List<Vector3f> contacts = calculateContactPoints(objectA, objectB, normal, depth);

            // Create and return a CollisionResult
            return new CollisionResult(colliding, normal, depth, contacts);
        } else {
            // No collision
            return new CollisionResult(false, null, 0.0f, null);
        }
    }

    public Vector3f calculateCollisionNormal(SATCollisionObject3D objectA, SATCollisionObject3D objectB) {
        Vector3f smallestOverlapAxis = new Vector3f();
        float smallestOverlap = Float.MAX_VALUE;

        List<Vector3f> axesA = objectA.getCollisionAxes();
        List<Vector3f> axesB = objectB.getCollisionAxes();

        for (Vector3f axis : axesA) {
            float overlap = calculateOverlapOnAxis(objectA, objectB, axis);

            if (overlap < 0) {
                // The objects are not colliding on this axis; early exit.
                return null;
            }

            if (overlap < smallestOverlap) {
                smallestOverlap = overlap;
                smallestOverlapAxis = axis;
            }
        }

        for (Vector3f axis : axesB) {
            float overlap = calculateOverlapOnAxis(objectA, objectB, axis);

            if (overlap < 0) {
                // The objects are not colliding on this axis; early exit.
                return null;
            }

            if (overlap < smallestOverlap) {
                smallestOverlap = overlap;
                smallestOverlapAxis = axis;
            }
        }

        return smallestOverlapAxis.normalize();
    }
    public float calculatePenetrationDepth(SATCollisionObject3D objectA, SATCollisionObject3D objectB, Vector3f normal) {
        // Calculate the projection of the objects' vertices onto the collision normal.
        float minA = calculateProjection(objectA, normal);
        float maxA = minA + objectA.getSizeOnAxis(normal);
        float minB = calculateProjection(objectB, normal);
        float maxB = minB + objectB.getSizeOnAxis(normal);

        // Calculate the penetration depth as the overlap between the projections.
        return Math.min(maxA, maxB) - Math.max(minA, minB);
    }
    public List<Vector3f> calculateContactPoints(SATCollisionObject3D objectA, SATCollisionObject3D objectB, Vector3f normal, float depth) {
        List<Vector3f> contacts = new ArrayList<>();

        if (depth > 0) {
            // Calculate contact points based on the penetration depth and normal.
            Vector3f contactPointA = objectA.getCenter().add(normal.scale(depth / 2.0f));
            Vector3f contactPointB = objectB.getCenter().subtract(normal.scale(depth / 2.0f));

            contacts.add(contactPointA);
            contacts.add(contactPointB);
        }

        return contacts;
    }
    public float calculateOverlapOnAxis(SATCollisionObject3D objectA, SATCollisionObject3D objectB, Vector3f axis) {
        // Project the vertices of objectA onto the axis.
        float minA = Float.POSITIVE_INFINITY;
        float maxA = Float.NEGATIVE_INFINITY;

        for (Vector3f vertex : objectA.getVertices()) {
            float projection = vertex.dot(axis);
            minA = Math.min(minA, projection);
            maxA = Math.max(maxA, projection);
        }

        // Project the vertices of objectB onto the axis.
        float minB = Float.POSITIVE_INFINITY;
        float maxB = Float.NEGATIVE_INFINITY;

        for (Vector3f vertex : objectB.getVertices()) {
            float projection = vertex.dot(axis);
            minB = Math.min(minB, projection);
            maxB = Math.max(maxB, projection);
        }

        // Calculate the overlap between the projections.
        float overlap = Math.min(maxA, maxB) - Math.max(minA, minB);

        return overlap;
    }
*/
}
