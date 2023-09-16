package collisionDetection.narrowPhase.cd;

import collisionDetection.narrowPhase.collisionResult.CollisionResult;
import collisionDetection.narrowPhase.sat.SATSupport;
import collisionDetection.primitive.*;
import collisionDetection.util.CollisionUtil;
import math.Vector3f;

import java.util.ArrayList;
import java.util.List;

public class CDSolver {

    public static CollisionResult solve(Plane plane, Cylinder cylinder) {
        return null;
    }

    public static CollisionResult solve(Plane plane, Sphere sphere) {
        return null;
    }

    public static CollisionResult solve(Plane plane, Capsule capsule) {
        return null;
    }

    public static CollisionResult solve(Plane plane, AABB aabb) {
        return null;
    }

    public static CollisionResult solve(Plane plane, OBB obb) {
        return null;
    }

    public static CollisionResult solve(Plane plane, Line line) {
        return null;
    }

    public static CollisionResult solve(Plane plane, Triangle triangle) {
        return null;
    }

    public static CollisionResult solve(Plane plane, ConvexPolyhedron convexPolyhedron) {
        // Initialize collision result
        CollisionResult result = new CollisionResult();
        result.setColliding(true);

        boolean colliding = false;
        List<Vector3f> contactPoints = new ArrayList<>();

        // Check if any vertex of the polyhedron is in front of the plane
        for (Vector3f vertex : convexPolyhedron.getVertices()) {
            if (plane.isPointInFront(vertex)) {
                colliding = true;

                // Calculate the intersection point with the plane
                Vector3f closestPointOnPlane = plane.closestPoint(vertex);
                contactPoints.add(closestPointOnPlane);
            }
        }

        if (colliding) {
            // Set collision result properties
            result.setNormal(plane.getNormal());

            // Calculate the penetration depth as the distance to the plane
            // from the closest vertex (assuming it's penetrating)
            float depth = Float.MAX_VALUE;
            for (Vector3f contactPoint : contactPoints) {
                float distance = plane.closestPoint(contactPoint).distance(contactPoint);
                depth = Math.min(depth, distance);
            }
            result.setDepth(depth);

            result.setContactPoints(contactPoints);
        }

        return result;
    }

    public static CollisionResult solve(Sphere sphere, SATSupport convexPolyhedron) {
        // Check if the sphere is completely inside the polyhedron
        if (sphere.isPointInside(convexPolyhedron.closestPoint(sphere.getCenter()))) {
            // Calculate the collision normal (pointing from polyhedron to sphere)
            Vector3f normal = sphere.getCenter().sub(convexPolyhedron.closestPoint(sphere.getCenter())).normalize();

            // Calculate the collision depth as the difference between sphere radius and distance to the closest point
            float depth = sphere.getRadius() - sphere.getCenter().distance(convexPolyhedron.closestPoint(sphere.getCenter()));

            return new CollisionResult(true, normal, depth, List.of(convexPolyhedron.closestPoint(sphere.getCenter())));
        }

        Vector3f normal = new Vector3f();
        float depth = 0;
        List<Vector3f> contactPoints = new ArrayList<>();
        // Handle the case where the sphere intersects with the polyhedron boundary
        // Iterate through the polyhedron's faces (triangles) and check for intersections
        for (int i = 0; i < convexPolyhedron.getVertices().size(); i++) {
            Vector3f v0 = convexPolyhedron.getVertices().get(i);
            Vector3f v1 = convexPolyhedron.getVertices().get((i + 1) % convexPolyhedron.getVertices().size());
            Vector3f faceNormal = v1.sub(v0).cross(sphere.getCenter().sub(v0)).normalize();

            // Check if the sphere intersects the plane defined by the face
            float distanceToPlane = faceNormal.dot(sphere.getCenter().sub(v0));
            if (Math.abs(distanceToPlane) <= sphere.getRadius()) {
                // Check if the intersection point is inside the face's boundaries
                if (CollisionUtil.isPointInsideFace(v0, v1, convexPolyhedron.getVertices())) {
                    // Calculate the intersection point on the plane
                    Vector3f intersectionPoint = sphere.getCenter().sub(faceNormal.mul(distanceToPlane));
                    normal.set(faceNormal);
                    depth = sphere.getRadius() - distanceToPlane;
                    contactPoints.add(intersectionPoint);

                }
            }
        }

        return new CollisionResult(true, normal, depth, contactPoints);
    }

    public static CollisionResult solve(Cylinder cylinder, SATSupport convexPolyhedron) {
        // Initialize collision result
        CollisionResult result = new CollisionResult();

        // Check if the cylinder is completely inside the polyhedron
        boolean cylinderInside = true;
        for (Vector3f vertex : convexPolyhedron.getVertices()) {
            if (!cylinder.isPointInside(vertex)) {
                cylinderInside = false;
                break;
            }
        }
        if (cylinderInside) {
            result.setColliding(true);
            // Calculate the collision normal (pointing from polyhedron to cylinder center)
            Vector3f normal = cylinder.getCenter().sub(convexPolyhedron.closestPoint(cylinder.getCenter())).normalize();
            // Calculate the collision depth as the distance between cylinder center and polyhedron
            float depth = cylinder.getCenter().distance(convexPolyhedron.closestPoint(cylinder.getCenter()));

            return new CollisionResult(true, normal, depth, List.of(convexPolyhedron.closestPoint(cylinder.getCenter())));
        }

        Vector3f normal = new Vector3f();
        float depth = 0;
        List<Vector3f> contactPoints = new ArrayList<>();
        // Handle the case where the cylinder intersects with the polyhedron boundary
        // Iterate through the polyhedron's faces (triangles) and check for intersections
        for (int i = 0; i < convexPolyhedron.getVertices().size(); i++) {
            Vector3f v0 = convexPolyhedron.getVertices().get(i);
            Vector3f v1 = convexPolyhedron.getVertices().get((i + 1) % convexPolyhedron.getVertices().size());
            Vector3f faceNormal = v1.sub(v0).cross(cylinder.getCenter().sub(v0)).normalize();

            // Check if the cylinder intersects the plane defined by the face
            float distanceToPlane = faceNormal.dot(cylinder.getCenter().sub(v0));
            if (Math.abs(distanceToPlane) <= cylinder.getRadius()) {

                // Check if the intersection point is inside the face's boundaries
                if (CollisionUtil.isPointInsideFace(v0, v1, convexPolyhedron.getVertices())) {
                    // Calculate the intersection point on the plane
                    Vector3f intersectionPoint = cylinder.getCenter().sub(faceNormal.mul(distanceToPlane));
                    normal.set(faceNormal);
                    depth = cylinder.getRadius() - distanceToPlane;
                    contactPoints.add(intersectionPoint);
                }
            }
        }

        return new CollisionResult(true, normal, depth, contactPoints);
    }

    public static CollisionResult solve(Capsule capsule, SATSupport convexPolyhedron) {

        // Check if any part of the capsule is completely inside the polyhedron
        boolean capsuleInside = true;
        if (!convexPolyhedron.isPointInside(capsule.getStart())) {
            capsuleInside = false;
            for (Vector3f vertex : convexPolyhedron.getVertices()) {
                if (!capsule.isPointInside(vertex)) {
                    break;
                }
            }
        }
        if (capsuleInside) {
            return new CollisionResult(true, Vector3f.Zero, 0, List.of(capsule.getStart(),
                    capsule.getEnd()));
        }

        Vector3f normal = new Vector3f();
        float depth = 0;
        List<Vector3f> contactPoints = new ArrayList<>();
        // Handle the case where the capsule intersects with the polyhedron boundary
        // Iterate through the polyhedron's faces (triangles) and check for intersections
        for (int i = 0; i < convexPolyhedron.getVertices().size(); i++) {
            Vector3f v0 = convexPolyhedron.getVertices().get(i);
            Vector3f v1 = convexPolyhedron.getVertices().get((i + 1) % convexPolyhedron.getVertices().size());
            Vector3f faceNormal = v1.sub(v0).cross(capsule.getStart().sub(v0)).normalize();

            // Check if the capsule intersects the plane defined by the face
            float distanceToPlane = faceNormal.dot(capsule.getStart().sub(v0));
            if (Math.abs(distanceToPlane) <= capsule.getRadius()) {

                // Check if the intersection point is inside the face's boundaries
                if (CollisionUtil.isPointInsideFace(v0, v1, convexPolyhedron.getVertices())) {
                    // Calculate the intersection point on the plane
                    Vector3f intersectionPoint = capsule.getStart().sub(faceNormal.mul(distanceToPlane));

                    normal.set(faceNormal);
                    depth = capsule.getRadius() - distanceToPlane;
                    // Add the intersection point as a contact point
                    contactPoints.add(intersectionPoint);

                }
            }
        }

        return new CollisionResult(true, normal, depth, contactPoints);
    }

    public static CollisionResult solve(Plane plane, SATSupport convexPolyhedron) {
        // Initialize collision result
        CollisionResult result = new CollisionResult();
        result.setColliding(true);

        boolean colliding = false;
        List<Vector3f> contactPoints = new ArrayList<>();

        // Check if any vertex of the polyhedron is in front of the plane
        for (Vector3f vertex : convexPolyhedron.getVertices()) {
            if (plane.isPointInFront(vertex)) {
                colliding = true;

                // Calculate the intersection point with the plane
                Vector3f closestPointOnPlane = plane.closestPoint(vertex);
                contactPoints.add(closestPointOnPlane);
            }
        }

        if (colliding) {
            // Set collision result properties
            result.setNormal(plane.getNormal());

            // Calculate the penetration depth as the distance to the plane
            // from the closest vertex (assuming it's penetrating)
            float depth = Float.MAX_VALUE;
            for (Vector3f contactPoint : contactPoints) {
                float distance = plane.closestPoint(contactPoint).distance(contactPoint);
                depth = Math.min(depth, distance);
            }
            result.setDepth(depth);

            result.setContactPoints(contactPoints);
        }

        return result;
    }


}
