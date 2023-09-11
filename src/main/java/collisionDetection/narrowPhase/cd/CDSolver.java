package collisionDetection.narrowPhase.cd;

import collisionDetection.narrowPhase.collisionResult.CollisionResult;
import collisionDetection.primitive.Capsule;
import collisionDetection.primitive.Cylinder;
import collisionDetection.primitive.Line;
import collisionDetection.primitive.Sphere;
import math.Vector3f;

import java.util.ArrayList;
import java.util.List;

public class CDSolver {
    public static CollisionResult solve(Line line, Capsule capsule) {

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

        List<Vector3f> contacts = new ArrayList<>();
        // Calculate the contact points (one on each side of the capsule's axis).
        Vector3f contactPoint1 = closestPointOnAxis.add(collisionNormal.mul(capsule.getRadius()));
        Vector3f contactPoint2 = closestPointOnAxis.sub(collisionNormal.mul(capsule.getRadius()));
        contacts.add(contactPoint1);
        contacts.add(contactPoint2);

        return new CollisionResult(true, collisionNormal, depth, contacts);
    }


    public static CollisionResult solve(Line line, Cylinder cylinder) {

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

        // Calculate the collision normal (from cylinder center to the closest point on the line)
        Vector3f collisionNormal = closestPointOnLine.sub(cylinder.getCenter()).normalize();

        // Calculate the penetration depth
        float penetrationDepth = cylinder.getRadius() - closestPointOnLine.distance(cylinder.getCenter());

        // Calculate and add the second contact point (reflection point)
        Vector3f reflectionPoint = closestPointOnLine.add(collisionNormal.mul(penetrationDepth));
        List<Vector3f> contacts = new ArrayList<>();
        contacts.add(closestPointOnLine);
        contacts.add(reflectionPoint);

        return new CollisionResult(true, collisionNormal, penetrationDepth, contacts);

    }

    public static CollisionResult solve(Line line, Sphere sphere) {

        // The line's start point is inside the sphere.
        // Calculate the closest point on the line to the sphere's center.
        Vector3f closestPointOnLine = line.closestPoint(sphere.getCenter());

        // Calculate the vector from the sphere's center to the closest point on the line.
        Vector3f toClosestPointOnLine = closestPointOnLine.sub(sphere.getCenter());

        // Calculate the collision normal (from the sphere's center to the closest point on the line).
        Vector3f collisionNormal = toClosestPointOnLine.normalize();

        // Calculate the collision depth (distance from the sphere's surface to the line).
        float penetrationDepth = sphere.getRadius() - toClosestPointOnLine.length();

        // Calculate the collision contact points.
        Vector3f collisionContact = line.closestPoint(sphere.getCenter());

        List<Vector3f> contacts = new ArrayList<>();
        contacts.add(closestPointOnLine);
        contacts.add(collisionContact);

        return new CollisionResult(true, collisionNormal, penetrationDepth, contacts);
    }


}
