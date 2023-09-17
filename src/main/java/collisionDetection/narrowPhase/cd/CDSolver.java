package collisionDetection.narrowPhase.cd;

import collisionDetection.narrowPhase.collisionResult.CollisionResult;
import collisionDetection.narrowPhase.sat.SATSupport;
import collisionDetection.primitive.*;
import collisionDetection.util.CollisionUtil;
import math.Vector3f;

import java.util.ArrayList;
import java.util.List;

public class CDSolver {

    public CollisionResult solve(Plane plane, Cylinder cylinder) {
        Vector3f cylinderCenter = cylinder.getCenter();

        // Check if the cylinder's center is inside the plane
        boolean isCylinderCenterInside = plane.isPointInside(cylinderCenter);

        // Calculate the closest point on the plane to the cylinder's center
        Vector3f closestPointOnPlane = plane.closestPoint(cylinderCenter);

        // Calculate the vector from the cylinder's center to the closest point on the plane
        Vector3f fromCylinderToPlane = closestPointOnPlane.sub(cylinderCenter);

        // Calculate the distance between the cylinder's center and the closest point on the plane
        float distanceToPlane = fromCylinderToPlane.length();

        if (isCylinderCenterInside) {

            return new CollisionResult(true, plane.getNormal(), distanceToPlane - cylinder.getRadius(), List.of(cylinderCenter));
        }
        // The cylinder's center is inside the cylinder, but not fully inside the plane
        // Calculate the contact point on the cylinder's surface
        Vector3f contactPoint = cylinder.closestPoint(closestPointOnPlane);

        return new CollisionResult(true, plane.getNormal(), distanceToPlane - cylinder.getRadius(), List.of(contactPoint));
    }


    public CollisionResult solve(Plane plane, Sphere sphere) {
        // Calculate the distance between the sphere's center and the plane
        float distanceToPlane = plane.getNormal().dot(sphere.getCenter()) + plane.getDistance();

        // Calculate collision depth
        float depth = sphere.getRadius() - Math.abs(distanceToPlane);

        // Calculate the collision normal (plane's normal)
        Vector3f collisionNormal = plane.getNormal();

        // Calculate the contact point (sphere's center projected onto the plane)
        Vector3f contactPoint = sphere.getCenter().sub(plane.getNormal().mul(distanceToPlane));

        // Create a CollisionResult object with collision information
        List<Vector3f> contactPoints = new ArrayList<>();
        contactPoints.add(contactPoint);
        return new CollisionResult(true, collisionNormal, depth, contactPoints);

    }


    public CollisionResult solve(Plane plane, Capsule capsule) {
        Vector3f axis = capsule.getEnd().sub(capsule.getStart());
        Vector3f vecToPoint = capsule.getStart().sub(plane.closestPoint(capsule.getStart()));

        float t = vecToPoint.dot(axis) / axis.dot(axis);
        t = Math.max(0, Math.min(1, t)); // Clamp t to [0, 1]

        Vector3f closestPointOnAxis = capsule.getStart().add(axis.mul(t));

        Vector3f closestPointOnSurface = closestPointOnAxis
                .add(vecToPoint.sub(closestPointOnAxis).normalize().mul(capsule.getRadius()));

        // Calculate the distance between the closest point on the capsule's surface and the plane
        float distanceToPlane = plane.getNormal().dot(closestPointOnSurface) + plane.getDistance();

        // Calculate collision depth
        float depth = capsule.getRadius() - Math.abs(distanceToPlane);

        // Create a CollisionResult object with collision information
        List<Vector3f> contactPoints = new ArrayList<>();
        contactPoints.add(closestPointOnSurface);
        return new CollisionResult(true, plane.getNormal(), depth, contactPoints);

    }


    public CollisionResult solve(Plane plane, AABB aabb) {

        // Calculate the center of the AABB
        Vector3f aabbCenter = aabb.getCenter();

        // Calculate the signed distance from the AABB center to the plane
        float signedDistance = plane.getNormal().dot(aabbCenter) - plane.getDistance();

        // Calculate the extent of the AABB along the plane normal
        float aabbExtent = (aabb.getMax().sub(aabb.getMin())).dot(plane.getNormal()) / 2.0f;

        // Calculate the penetration depth
        float depth = aabbExtent - Math.abs(signedDistance);

        // Calculate the collision normal (opposite of the plane's normal)
        Vector3f collisionNormal = plane.getNormal().negate();

        // Calculate the contact point on the AABB
        Vector3f contactPoint = aabbCenter.sub(collisionNormal.mul(aabbExtent));

        return new CollisionResult(true, collisionNormal, depth, List.of(contactPoint));
    }


    public CollisionResult solve(Plane plane, OBB obb) {

        // Calculate the signed distance from the OBB center to the plane
        Vector3f obbCenter = obb.getCenter();
        float signedDistance = plane.getNormal().dot(obbCenter) - plane.getDistance();

        // Calculate the projection radius of the OBB onto the plane normal
        float projectionRadius = 0.0f;
        for (int i = 0; i < 3; i++) {
            float axisProjection = Math.abs(obb.getAxis().get(i).dot(plane.getNormal()));
            projectionRadius += obb.getHalfExtents().get(i) * axisProjection;
        }

        // Calculate the penetration depth
        float depth = projectionRadius - Math.abs(signedDistance);

        // Calculate the collision normal (opposite of the plane's normal)
        Vector3f collisionNormal = plane.getNormal().negate();

        // Calculate the contact point on the OBB
        Vector3f contactPoint = obb.closestPoint(obbCenter.add(plane.getNormal().mul(signedDistance)));

        return new CollisionResult(true, collisionNormal, depth, List.of(contactPoint));
    }


    public CollisionResult solve(Plane plane, Line line) {

        // Calculate the signed distances from the line's start and end points to the plane
        float startDistance = plane.getNormal().dot(line.getStart()) + plane.getDistance();
        float endDistance = plane.getNormal().dot(line.getEnd()) + plane.getDistance();

        // Calculate the intersection point with the plane
        float t = startDistance / (startDistance - endDistance);
        Vector3f intersectionPoint = line.getStart().lerp(line.getEnd(), t);

        // Calculate the collision normal (opposite of the plane's normal)
        Vector3f collisionNormal = plane.getNormal().negate();

        // Calculate the penetration depth (distance from intersection to plane)
        float depth = Math.abs(startDistance) / plane.getNormal().length();

        return new CollisionResult(true, collisionNormal, depth, List.of(intersectionPoint));
    }


    public CollisionResult solve(Plane plane, Triangle triangle) {
        // Check if any of the triangle's vertices are in front of the plane
        boolean v1InFront = plane.isPointInFront(triangle.getVertex1());
        boolean v2InFront = plane.isPointInFront(triangle.getVertex2());
        boolean v3InFront = plane.isPointInFront(triangle.getVertex3());

        // Calculate the intersection point of the triangle with the plane
        Vector3f intersectionPoint = new Vector3f();

        // Check if each triangle edge intersects with the plane
        if (v1InFront != v2InFront) {
            intersectionPoint = CollisionUtil.closestPointBetweenTwoLines(
                    triangle.getVertex1(), triangle.getVertex2(),
                    triangle.getVertex1(), triangle.getVertex3()
            );
        } else if (v2InFront != v3InFront) {
            intersectionPoint = CollisionUtil.closestPointBetweenTwoLines(
                    triangle.getVertex2(), triangle.getVertex3(),
                    triangle.getVertex2(), triangle.getVertex1()
            );
        } else {
            intersectionPoint = CollisionUtil.closestPointBetweenTwoLines(
                    triangle.getVertex3(), triangle.getVertex1(),
                    triangle.getVertex3(), triangle.getVertex2()
            );
        }

        // Calculate the collision normal (opposite of the plane's normal)
        Vector3f collisionNormal = plane.getNormal().negate();

        // Calculate the penetration depth (distance from the intersection point to the plane)
        float depth = Math.abs(plane.distanceToPoint(intersectionPoint));

        return new CollisionResult(true, collisionNormal, depth, List.of(intersectionPoint));
    }


    public CollisionResult solve(Plane plane, ConvexPolyhedron convexPolyhedron) {
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

    public CollisionResult solve(Sphere sphere, SATSupport convexPolyhedron) {
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

    public CollisionResult solve(Cylinder cylinder, SATSupport convexPolyhedron) {
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

    public CollisionResult solve(Capsule capsule, SATSupport convexPolyhedron) {

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

    public CollisionResult solve(Plane plane, SATSupport convexPolyhedron) {
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
