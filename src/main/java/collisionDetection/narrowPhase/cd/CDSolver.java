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

    public static CollisionResult solve(Sphere sphere, Triangle triangle) {

        // Calculate collision details
        List<Vector3f> contactPoints = new ArrayList<>();

        // Calculate the closest point on the triangle to the sphere's center
        Vector3f closestPointOnTriangle = triangle.closestPoint(sphere.getCenter());

        // Calculate the collision normal (from sphere center to the closest point on triangle)
        Vector3f collisionNormal = closestPointOnTriangle.sub(sphere.getCenter()).normalize();

        // Calculate the penetration depth (distance from sphere center to the closest point on triangle minus sphere radius)
        float penetrationDepth = closestPointOnTriangle.distance(sphere.getCenter()) - sphere.getRadius();

        // Add the closest point on the triangle to the contact points list
        contactPoints.add(closestPointOnTriangle);

        // Find the second-closest point on the triangle to the sphere's center
        Vector3f secondClosestPointOnTriangle = triangle.closestPoint(sphere.getCenter().add(collisionNormal.mul(sphere.getRadius())));

        // Add the second-closest point on the triangle to the contact points list
        contactPoints.add(secondClosestPointOnTriangle);

        // Create and return a CollisionResult object
        return new CollisionResult(true, collisionNormal, penetrationDepth, contactPoints);
    }

    public static CollisionResult solve(Sphere sphere, OBB obb) {

        // Calculate the closest point on the OBB to the sphere's center
        Vector3f closestPoint = obb.closestPoint(sphere.getCenter());

        // Calculate the vector from the sphere's center to the closest point
        Vector3f sphereToClosest = closestPoint.sub(sphere.getCenter());

        // Calculate the distance between the sphere's center and the closest point
        float distance = sphereToClosest.length();

        List<Vector3f> contactPoints = new ArrayList<>();

        // Calculate the collision normal (from the OBB center to the sphere center)
        Vector3f collisionNormal = sphereToClosest.normalize();

        // Calculate the penetration depth
        float penetrationDepth = sphere.getRadius() - distance;

        // Calculate the first contact point on the sphere's surface
        Vector3f contactPoint1 = sphere.getCenter().add(collisionNormal.mul(sphere.getRadius()));
        contactPoints.add(contactPoint1);

        // Calculate the second contact point on the sphere's surface (opposite side)
        Vector3f contactPoint2 = sphere.getCenter().sub(collisionNormal.mul(sphere.getRadius()));
        contactPoints.add(contactPoint2);

        return new CollisionResult(true, collisionNormal, penetrationDepth, contactPoints);

    }

    public static CollisionResult solve(Sphere sphere, Capsule capsule) {

        List<Vector3f> contactPoints = new ArrayList<>();

        // Compute the collision normal as the normalized vector from the sphere center to the capsule's closest point
        Vector3f closestPoint = capsule.closestPoint(sphere.getCenter());
        Vector3f collisionNormal = closestPoint.sub(sphere.getCenter()).normalize();

        // Compute the penetration depth (distance from sphere center to capsule's surface)
        float depth = sphere.getRadius() - sphere.getCenter().distance(closestPoint);

        // Compute the first contact point (the closest point on the sphere's surface)
        Vector3f contactPoint1 = sphere.getCenter().add(collisionNormal.mul(sphere.getRadius()));
        contactPoints.add(contactPoint1);

        // To compute the second contact point, we reflect the first contact point across the collision normal
        Vector3f reflection = collisionNormal.mul(2.0f * depth); // Reflect by twice the penetration depth
        Vector3f contactPoint2 = contactPoint1.add(reflection);
        contactPoints.add(contactPoint2);

        return new CollisionResult(true, collisionNormal, depth, contactPoints);
    }

    public static CollisionResult solve(Sphere sphere, Cylinder cylinder) {

        List<Vector3f> contactPoints = new ArrayList<>();
        // Calculate the closest point on the cylinder to the sphere's center
        Vector3f closestPointOnCylinder = cylinder.closestPoint(sphere.getCenter());

        // Calculate the vector from the sphere's center to the closest point on the cylinder
        Vector3f collisionVector = closestPointOnCylinder.sub(sphere.getCenter());

        // Calculate the distance between the sphere's center and the closest point on the cylinder
        float distance = collisionVector.length();

        // Calculate the collision normal (points from cylinder to sphere)
        Vector3f collisionNormal = collisionVector.normalize();

        // Calculate the penetration depth
        float penetrationDepth = sphere.getRadius() - distance;

        // Calculate two contact points on the sphere's surface
        Vector3f contactPoint1 = sphere.getCenter().add(collisionNormal.mul(sphere.getRadius() - penetrationDepth));
        Vector3f contactPoint2 = sphere.getCenter().add(collisionNormal.mul(sphere.getRadius()));
        contactPoints.add(contactPoint1);
        contactPoints.add(contactPoint2);

        return new CollisionResult(true, collisionNormal, penetrationDepth, contactPoints);
    }

    public static CollisionResult solve(Sphere sphere, ConvexPolyhedron convexPolyhedron) {

        List<Vector3f> contactPoints = new ArrayList<>();
        // Calculate the normal vector from the sphere's center to the closest point on the polyhedron
        Vector3f closestPoint = convexPolyhedron.closestPoint(sphere.getCenter());
        Vector3f collisionNormal = sphere.getCenter().sub(closestPoint).normalize();

        // Calculate the depth (penetration distance)
        float depth = sphere.getRadius() - sphere.getCenter().distance(closestPoint);

        // Find two contact points by moving along and away from the normal
        Vector3f contactPoint1 = closestPoint.add(collisionNormal.mul(sphere.getRadius()));
        Vector3f contactPoint2 = closestPoint.sub(collisionNormal.mul(sphere.getRadius()));

        // Add both contact points to the result
        contactPoints.add(contactPoint1);
        contactPoints.add(contactPoint2);
        return new CollisionResult(true, collisionNormal, depth, contactPoints);
    }

    public static CollisionResult solve(Cylinder cylinder, ConvexPolyhedron convexPolyhedron) {
        // Initialize collision result variables
        boolean colliding = false;
        Vector3f collisionNormal = new Vector3f();
        float penetrationDepth = 0.0f;
        List<Vector3f> contactPoints = new ArrayList<>();

        // Check if any point on the cylinder's surface is inside the polyhedron
        for (float angle = 0.0f; angle < 360.0f; angle += 10.0f) {
            // Calculate a point on the cylinder's surface at the given angle
            float x = cylinder.getRadius() * (float) Math.cos(Math.toRadians(angle)) + cylinder.getCenter().getX();
            float z = cylinder.getRadius() * (float) Math.sin(Math.toRadians(angle)) + cylinder.getCenter().getZ();
            Vector3f cylinderSurfacePoint = new Vector3f(x, cylinder.getCenter().getY(), z);

            // Check if the cylinder's surface point is inside the polyhedron
            if (convexPolyhedron.isPointInside(cylinderSurfacePoint)) {
                colliding = true;

                // Calculate the collision normal (pointing from the polyhedron to the cylinder)
                collisionNormal = cylinderSurfacePoint.sub(convexPolyhedron.closestPoint(cylinderSurfacePoint)).normalize();

                // Calculate the penetration depth (distance from the cylinder's surface to the polyhedron)
                penetrationDepth = cylinder.getRadius() - cylinder.getCenter().distance(cylinderSurfacePoint);

                // Add the contact point to the list
                contactPoints.add(cylinderSurfacePoint);
            }
        }

        // Check if any point on the polyhedron is inside the cylinder
        for (Vector3f vertex : convexPolyhedron.getVertices()) {
            if (cylinder.isPointInside(vertex)) {
                colliding = true;

                // Calculate the collision normal (pointing from the cylinder to the polyhedron)
                collisionNormal = vertex.sub(cylinder.closestPoint(vertex)).normalize();

                // Calculate the penetration depth (distance from the polyhedron's surface to the cylinder)
                penetrationDepth = cylinder.getRadius() - cylinder.getCenter().distance(vertex);

                // Add the contact point to the list
                contactPoints.add(vertex);
            }
        }

        // Create and return the collision result
        return new CollisionResult(colliding, collisionNormal, penetrationDepth, contactPoints);
    }

    public static CollisionResult solve(Capsule capsule, ConvexPolyhedron convexPolyhedron) {
        // Find the closest points between the capsule and the polyhedron
        Vector3f closestPointOnCapsule = capsule.closestPoint(convexPolyhedron.closestPoint(capsule.getStart()));
        Vector3f closestPointOnPolyhedron = convexPolyhedron.closestPoint(capsule.getStart());

        // Calculate the collision normal
        Vector3f normal = closestPointOnPolyhedron.sub(closestPointOnCapsule).normalize();

        // Calculate the depth of the collision
        float depth = normal.dot(closestPointOnCapsule.sub(capsule.getStart()));

        // Collect all the contact points
        List<Vector3f> contactPoints = new ArrayList<>();
        for (int i = 0; i < convexPolyhedron.getVertices().size(); i++) {
            Vector3f vertex = convexPolyhedron.getVertices().get(i);
            Vector3f contactPoint = capsule.closestPoint(vertex);
            if (capsule.isPointInside(contactPoint)) {
                contactPoints.add(contactPoint);
            }
        }

        return new CollisionResult(true, normal, depth, contactPoints);
    }

}
