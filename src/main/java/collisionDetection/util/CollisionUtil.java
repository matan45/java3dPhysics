package collisionDetection.util;

import collisionDetection.narrowPhase.gjk.GJKSupport;
import collisionDetection.narrowPhase.sat.SATSupport;
import collisionDetection.primitive.*;
import math.Vector3f;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class CollisionUtil {

    public static Vector3f support(GJKSupport shape1, GJKSupport shape2, Vector3f direction) {
        Vector3f pointA = shape1.support(direction);
        Vector3f pointB = shape2.support(direction.negate());
        return pointA.sub(pointB);
    }

    // Helper method to calculate the closest point on a line segment to an AABB
    public static Vector3f closestPointOnLineToAABB(AABB aabb, Line line) {
        Vector3f start = line.getStart();
        Vector3f end = line.getEnd();
        Vector3f min = aabb.getMin();
        // Calculate the direction of the line segment
        Vector3f segmentDirection = end.sub(start);

        // Calculate the parameter along the segment where the closest point lies
        float t = segmentDirection.dot(min.sub(start)) / segmentDirection.lengthSquared();

        // Clamp the parameter to ensure the point is within the segment
        t = Math.max(0, Math.min(1, t));

        // Calculate the closest point on the segment to the AABB
        return start.add(segmentDirection.mul(t));
    }

    public static float calculateDistanceToPlane(Vector3f point, Vector3f normal, float constant) {
        return normal.dot(point) + constant / normal.length();
    }


    public static Vector3f calculateIntersectionPoint(Capsule capsule, Triangle triangle, Vector3f triangleNormal, float dotProduct) {
        // Calculate the parameter along the capsule line where the intersection occurs.
        float t = triangleNormal.dot(triangle.getVertex1().sub(capsule.getStart())) / dotProduct;

        // Calculate the intersection point using the parameter.

        return capsule.getStart().add(capsule.getEnd().sub(capsule.getStart()).mul(t));
    }

    public static boolean cylinderOBBCapsFacesIntersect(Cylinder cylinder, OBB obb) {

        // Calculate the distances from cylinder caps to OBB faces
        float distanceTopCap = Math.abs(obb.getAxis().get(1).dot(cylinder.getCenter().sub(obb.getCenter()))) + obb.getHalfExtents().y;
        float distanceBottomCap = Math.abs(obb.getAxis().get(1).dot(cylinder.getCenter().sub(obb.getCenter()))) - obb.getHalfExtents().y;

        // Check if cylinder top cap intersects with OBB top face
        if (distanceTopCap > cylinder.getHeight() * 0.5f) {
            return false;
        }

        // Check if cylinder bottom cap intersects with OBB bottom face
        return !(distanceBottomCap > cylinder.getHeight() * 0.5f);// Caps and faces intersect
    }

    public static boolean isLineCollidingWithCylinderBody(Line line, Cylinder cylinder) {
        Vector3f start = line.getStart();
        Vector3f end = line.getEnd();
        // Check if either end of the segment is within the cylinder's body
        boolean startInside = cylinder.isPointInside(start);
        boolean endInside = cylinder.isPointInside(end);

        // If the segment crosses the cylinder's central axis, there's a collision
        if (startInside || endInside) {
            return true;
        }

        // Check if the segment intersects the cylinder's body by checking the distance
        // of the segment's closest point to the cylinder's central axis
        Vector3f segmentDirection = end.sub(start);
        float t = segmentDirection.dot(cylinder.getCenter().sub(start)) / segmentDirection.lengthSquared();
        t = Math.max(0, Math.min(1, t));
        Vector3f closestPointOnSegment = start.add(segmentDirection.mul(t));

        return closestPointOnSegment.sub(cylinder.getCenter()).lengthSquared() <= cylinder.getRadius() * cylinder.getRadius();
    }

    public static Vector3f barycentric(Vector3f p, Triangle t) {
        Vector3f ap = p.sub(t.getVertex1());
        Vector3f bp = p.sub(t.getVertex2());
        Vector3f cp = p.sub(t.getVertex3());

        Vector3f ab = t.getVertex2().sub(t.getVertex1());
        Vector3f ac = t.getVertex3().sub(t.getVertex1());
        Vector3f bc = t.getVertex3().sub(t.getVertex2());
        Vector3f cb = t.getVertex2().sub(t.getVertex3());
        Vector3f ca = t.getVertex1().sub(t.getVertex3());

        Vector3f v = ab.sub(Vector3f.project(ab, cb));
        float a = 1.0f - (v.dot(ap) / v.dot(ab));

        v = bc.sub(Vector3f.project(bc, ac));
        float b = 1.0f - (v.dot(bp) / v.dot(bc));

        v = ca.sub(Vector3f.project(ca, ab));
        float c = 1.0f - (v.dot(cp) / v.dot(ca));

        return new Vector3f(a, b, c);
    }

    public static Set<Vector3f> combineAxis(SATSupport shape1, SATSupport shape2) {
        Set<Vector3f> allAxis = new HashSet<>();
        List<Vector3f> shape1Axis = shape1.getAxis();
        List<Vector3f> shape2Axis = shape2.getAxis();

        for (Vector3f axis1 : shape1Axis) {
            for (Vector3f axis2 : shape2Axis) {
                allAxis.add(axis1.cross(axis2).normalize());
            }
        }
        allAxis.addAll(shape1Axis);
        allAxis.addAll(shape2Axis);

        return allAxis;
    }

}
