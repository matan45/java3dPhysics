package collisionDetection.narrowPhase.cd;

import collisionDetection.narrowPhase.sat.SATSupport;
import collisionDetection.primitive.*;
import collisionDetection.primitive.terrain.TerrainShape;
import collisionDetection.util.CollisionUtil;
import math.Vector3f;

import java.util.List;

import static math.Const.EPSILON;

public class CDSatGjk {


    public static boolean isCollide(Plane plane, SATSupport convexPolyhedron) {
        // Check if any vertex of the polyhedron is in front of the plane
        for (Vector3f vertex : convexPolyhedron.getVertices()) {
            if (plane.isPointInFront(vertex)) {
                return true;
            }
        }

        return false;
    }

    public static boolean isCollide(Sphere sphere, SATSupport convexPolyhedron) {
        // Check if the sphere is completely outside the polyhedron
        if (!convexPolyhedron.isPointInside(sphere.getCenter())) {
            return false;
        }

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
                    return true;
                }
            }
        }

        return false;
    }

    public static boolean isCollide(Cylinder cylinder, SATSupport convexPolyhedron) {
        // Check if the cylinder is completely outside the polyhedron
        if (!convexPolyhedron.isPointInside(cylinder.getCenter())) {
            return false;
        }

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
                    return true;
                }
            }
        }

        return false;
    }

    public static boolean isCollide(Capsule capsule, SATSupport convexPolyhedron) {

        // Check if the capsule is completely outside the polyhedron
        if (!convexPolyhedron.isPointInside(capsule.getStart()) &&
                !convexPolyhedron.isPointInside(capsule.getEnd())) {
            return false;
        }

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
                    return true;
                }
            }
        }

        return false;
    }


    public static boolean isCollide(Plane plane, Cylinder cylinder) {
        // Calculate the projection of the cylinder's center onto the plane's normal
        float distanceToPlane = cylinder.getCenter().dot(plane.getNormal()) - plane.getDistance();

        // Calculate the distance between the center of the cylinder's top cap and the plane
        float topCapDistance = Math.abs(distanceToPlane) - cylinder.getHeight() / 2.0f;

        // Calculate the distance between the center of the cylinder's bottom cap and the plane
        float bottomCapDistance = Math.abs(distanceToPlane) + cylinder.getHeight() / 2.0f;

        // If both the top and bottom caps are on the same side of the plane's normal, there's no collision
        if (topCapDistance > cylinder.getRadius() && bottomCapDistance > cylinder.getRadius()) {
            return false;
        }

        // If the center of the cylinder is on the plane or the top/bottom caps intersect the plane, there's a collision
        return Math.abs(distanceToPlane) <= cylinder.getRadius();
    }

    public static boolean isCollide(Plane plane, Sphere sphere) {
        // Calculate the signed distance from the center of the sphere to the plane
        float signedDistance = plane.getNormal().dot(sphere.getCenter()) - plane.getDistance();

        // If the signed distance is less than or equal to the sphere's radius, there is a collision
        return Math.abs(signedDistance) <= sphere.getRadius();
    }

    public static boolean isCollide(Plane plane, Capsule capsule) {
        // Calculate the direction of the capsule's axis
        Vector3f capsuleAxis = capsule.getEnd().sub(capsule.getStart());

        // Calculate the projection of the capsule's axis onto the plane's normal
        float projection = capsuleAxis.dot(plane.getNormal());

        // If the capsule's axis is almost parallel to the plane, there's no collision
        if (Math.abs(projection) < EPSILON) {
            return false;
        }

        // Calculate the distance from the start of the capsule to the plane
        float distanceToStart = (capsule.getStart().dot(plane.getNormal()) - plane.getDistance()) / projection;

        // Calculate the distance from the end of the capsule to the plane
        float distanceToEnd = (capsule.getEnd().dot(plane.getNormal()) - plane.getDistance()) / projection;

        // If both the start and end of the capsule are on the same side of the plane, there's no collision
        return (!(distanceToStart < 0) || !(distanceToEnd < 0)) && (!(distanceToStart > 1) || !(distanceToEnd > 1));
    }

    public static boolean isCollide(Plane plane, AABB aabb) {
        // Calculate the half-extents of the AABB
        Vector3f halfExtents = aabb.getMax().sub(aabb.getMin()).div(2.0f);

        // Calculate the center of the AABB
        Vector3f center = aabb.getMin().add(halfExtents);

        // Calculate the distance from the center of the AABB to the plane
        float distance = plane.getNormal().dot(center) + plane.getDistance();

        // Calculate the projection of the half-extents onto the plane normal
        float projection = halfExtents.x * Math.abs(plane.getNormal().x) +
                halfExtents.y * Math.abs(plane.getNormal().y) +
                halfExtents.z * Math.abs(plane.getNormal().z);

        // Check if the AABB is entirely in front of or behind the plane
        // The AABB is entirely behind the plane
        if (distance > projection) {
            // The AABB is entirely in front of the plane
            return false;
        } else return !(distance < -projection);
    }

    public static boolean isCollide(Plane plane, OBB obb) {
        // Calculate the signed distance from the OBB's center to the plane
        float signedDistance = plane.getNormal().dot(obb.getCenter()) - plane.getDistance();

        // Calculate the projection radius of the OBB onto the plane's normal
        float projectionRadius = 0.0f;
        for (int i = 0; i < 3; i++) {
            projectionRadius += Math.abs(obb.getAxis().get(i).dot(plane.getNormal())) * obb.getHalfExtents().get(i);
        }

        // Check if the OBB is entirely in front of or behind the plane
        // OBB intersects the plane
        if (signedDistance > projectionRadius) {
            return false; // OBB is entirely in front of the plane
        } else return !(signedDistance < -projectionRadius); // OBB is entirely behind the plane
    }

    public static boolean isCollide(Plane plane, Line line) {
        // Calculate the direction vector of the line
        Vector3f lineDirection = line.getEnd().sub(line.getStart());

        // Check if the line is parallel to the plane (dot product of their directions is close to 0)
        float dotProduct = lineDirection.dot(plane.getNormal());
        if (Math.abs(dotProduct) < EPSILON) {
            return false; // Line is parallel to the plane and does not intersect
        }

        // Calculate the point of intersection between the line and the plane
        Vector3f lineStart = line.getStart();
        Vector3f planeNormal = plane.getNormal();
        float planeDistance = plane.getDistance();

        // Calculate the parameter t for the line equation (lineStart + t * lineDirection)
        float t = (planeDistance - lineStart.dot(planeNormal)) / dotProduct;

        // Check if the intersection point is within the line segment
        // Line intersects with the plane, but not within the line segment
        return t >= 0 && t <= 1; // Line intersects with the plane within the line segment
    }

    public static boolean isCollide(Plane plane, Triangle triangle) {
        List<Vector3f> vertices = triangle.getVertices();
        boolean allVerticesInFront = true;
        boolean allVerticesBehind = true;

        for (Vector3f vertex : vertices) {
            if (plane.isPointInFront(vertex)) {
                allVerticesBehind = false;
            } else {
                allVerticesInFront = false;
            }
        }

        return !(allVerticesInFront || allVerticesBehind);
    }

    public static boolean isCollide(Plane plane, ConvexPolyhedron convexPolyhedron) {
        // Check if any vertex of the polyhedron is in front of the plane
        for (Vector3f vertex : convexPolyhedron.getVertices()) {
            if (plane.isPointInFront(vertex)) {
                return true;
            }
        }

        return false;
    }

    public static boolean isCollide(Line line, TerrainShape terrainShape) {
        return terrainShape.isCollide(line);
    }

}
