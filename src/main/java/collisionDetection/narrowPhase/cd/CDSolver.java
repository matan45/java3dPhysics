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

        // Calculate the closest points on the line and the capsule
        Vector3f closestPointOnLine = line.closestPoint(capsule.getStart());
        Vector3f closestPointOnCapsule = capsule.closestPoint(closestPointOnLine);

        // Calculate the vector between the two closest points
        Vector3f collisionVector = closestPointOnLine.sub(closestPointOnCapsule);
        float distanceSquared = collisionVector.lengthSquared();

        // Check if there's a collision by comparing the distance to the capsule's radius
        float radiusSquared = capsule.getRadius() * capsule.getRadius();
        if (distanceSquared <= radiusSquared) {
            // There is a collision
            result.setColliding(true);
            result.setDepth((float) Math.sqrt(radiusSquared - distanceSquared));

            // Calculate the collision normal
            Vector3f normal = collisionVector.normalize();
            result.setNormal(normal);

            // Calculate the contact points
            List<Vector3f> contacts = new ArrayList<>();
            Vector3f contactPoint1 = closestPointOnCapsule.add(normal.mul(result.getDepth() * 0.5f));
            Vector3f contactPoint2 = closestPointOnCapsule.sub(normal.mul(result.getDepth() * 0.5f));
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
