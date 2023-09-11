package collisionDetection.narrowPhase.cd;

import collisionDetection.narrowPhase.collisionResult.CollisionResult;
import collisionDetection.primitive.Capsule;
import collisionDetection.primitive.Cylinder;
import collisionDetection.primitive.Line;
import math.Vector3f;

import java.util.ArrayList;
import java.util.List;

public class CDSolver {
    public static CollisionResult solve(Line line, Capsule capsule) {
        CollisionResult result = new CollisionResult();

        // First, calculate the closest point on the line to the capsule's axis.
        Vector3f axis = capsule.getEnd().sub(capsule.getStart());
        Vector3f vecToLineStart = line.getStart().sub(capsule.getStart());

        float t = vecToLineStart.dot(axis) / axis.dot(axis);
        t = Math.max(0, Math.min(1, t)); // Clamp t to [0, 1]

        Vector3f closestPointOnAxis = capsule.getStart().add(axis.mul(t));

        // Then, calculate the closest point on the line to the closest point on the capsule's axis.
        Vector3f closestPointOnLine = line.closestPoint(closestPointOnAxis);

        // Calculate the collision normal as the normalized vector between the two closest points.
        Vector3f collisionNormal = closestPointOnLine.sub(closestPointOnAxis).normalize();

        // Calculate the depth of penetration as the difference between the capsule's radius and the distance between the two closest points.
        float depth = capsule.getRadius() - closestPointOnLine.distance(closestPointOnAxis);

        // Check if the depth is positive, indicating a collision.
        if (depth > 0) {
            result.setColliding(true);
            result.setNormal(collisionNormal);
            result.setDepth(depth);
            List<Vector3f> contacts = new ArrayList<>();
            // Calculate the contact points (one on each side of the capsule's axis).
            Vector3f contactPoint1 = closestPointOnAxis.add(collisionNormal.mul(capsule.getRadius()));
            Vector3f contactPoint2 = closestPointOnAxis.sub(collisionNormal.mul(capsule.getRadius()));
            contacts.add(contactPoint1);
            contacts.add(contactPoint2);
            result.setContacts(contacts);
        }

        return result;
    }


    public static CollisionResult solve(Line line, Cylinder cylinder) {
        // Initialize the CollisionResult with default values
        CollisionResult result = new CollisionResult();

        // Calculate the direction vector of the line segment
        Vector3f lineDirection = line.getEnd().sub(line.getStart());

        // Calculate the vector from the cylinder's center to the start of the line
        Vector3f cylinderToLineStart = line.getStart().sub(cylinder.getCenter());

        // Project the cylinder-to-line-start vector onto the line direction
        float t = cylinderToLineStart.dot(lineDirection) / lineDirection.lengthSquared();

        // Ensure t is within the valid range [0, 1] for a point on the line segment
        t = Math.max(0, Math.min(1, t));

        // Calculate the closest point on the line segment to the cylinder's center
        Vector3f closestPointOnLine = line.getStart().add(lineDirection.mul(t));

        // Check if the closest point on the line is inside the cylinder's bounds
        if (cylinder.isPointInside(closestPointOnLine)) {
            // Calculate the collision normal (from cylinder center to the closest point on the line)
            Vector3f collisionNormal = closestPointOnLine.sub(cylinder.getCenter()).normalize();

            // Calculate the penetration depth
            float penetrationDepth = cylinder.getRadius() - closestPointOnLine.distance(cylinder.getCenter());

            // Populate the CollisionResult
            result.setColliding(true);
            result.setNormal(collisionNormal);
            result.setDepth(penetrationDepth);
            result.getContacts().add(closestPointOnLine);

            return result;
        }

        // If no collision, return the default CollisionResult indicating no collision
        return result;
    }

}
