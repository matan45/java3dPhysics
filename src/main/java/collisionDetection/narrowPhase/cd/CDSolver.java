package collisionDetection.narrowPhase.cd;

import collisionDetection.narrowPhase.collisionResult.CollisionResult;
import collisionDetection.primitive.*;
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

    public static CollisionResult solve(Line line, Plane plane) {

        Vector3f lineStart = line.getStart();
        Vector3f lineEnd = line.getEnd();

        // Calculate the direction of the line segment
        Vector3f lineDirection = lineEnd.sub(lineStart);

        // Calculate the dot product between the line's direction and the plane's normal
        float dotProduct = lineDirection.dot(plane.getNormal());

        // Calculate the parameter t for the line's intersection with the plane
        float t = (plane.getDistance() - lineStart.dot(plane.getNormal())) / dotProduct;

        // Calculate the intersection point
        Vector3f intersectionPoint = lineStart.add(lineDirection.mul(t));

        // Now, find the second contact point by reflecting the line's direction
        // with respect to the plane's normal.
        Vector3f reflectedDirection = lineDirection.sub(plane.getNormal().mul(2.0f * lineDirection.dot(plane.getNormal())));
        Vector3f secondIntersectionPoint = intersectionPoint.add(reflectedDirection);

        List<Vector3f> contacts = new ArrayList<>();
        // Add the second contact point
        contacts.add(secondIntersectionPoint);
        contacts.add(intersectionPoint);


        return new CollisionResult(true, plane.getNormal(), t, contacts);
    }

    public static CollisionResult solve(Sphere sphere, AABB aabb) {

        // Calculate the closest point on the AABB to the sphere center
        Vector3f closestPoint = aabb.closestPoint(sphere.getCenter());

        // Calculate the vector from the sphere center to the closest point on the AABB
        Vector3f sphereToAABB = closestPoint.sub(sphere.getCenter());

        // Calculate the distance between the sphere center and the closest point
        float distance = sphereToAABB.length();

        // Calculate the collision normal (pointing from AABB to sphere)
        Vector3f normal = sphereToAABB.normalize();

        // Calculate the penetration depth
        float penetrationDepth = sphere.getRadius() - distance;

        List<Vector3f> contacts = new ArrayList<>();

        // Calculate the second contact point (opposite direction from the first contact)
        Vector3f secondContact = closestPoint.add(normal.mul(penetrationDepth));
        contacts.add(secondContact);
        // Add the contact point (the closest point on AABB) to the collision result
        contacts.add(closestPoint);

        return new CollisionResult(true, normal, penetrationDepth, contacts);
    }

    public static CollisionResult solve(Sphere sphere, Plane plane) {

        // Calculate the distance between the center of the sphere and the plane.
        float distance = plane.getNormal().dot(sphere.getCenter()) - plane.getDistance();

        // Calculate the collision normal (opposite of the plane's normal if outside the plane).
        Vector3f normal = (distance > 0) ? plane.getNormal() : plane.getNormal().negate();

        List<Vector3f> contacts = new ArrayList<>();
        // Calculate the two contact points on the sphere's surface closest to the plane.
        Vector3f contactPoint1 = sphere.closestPoint(plane.closestPoint(sphere.getCenter()));
        Vector3f contactPoint2 = sphere.getCenter().add(normal.mul(sphere.getRadius())); // Opposite side contact point
        contacts.add(contactPoint1);
        contacts.add(contactPoint2);

        // Calculate the penetration depth
        float penetrationDepth = Math.abs(distance) - sphere.getRadius();

        return new CollisionResult(true, normal, penetrationDepth, contacts);
    }


}
