package collisionDetection.util;

import collisionDetection.narrowPhase.gjk.GJKSupport;
import collisionDetection.primitive.*;
import math.Vector3f;

import java.util.List;

import static math.Const.EPSILON;

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

    public static int direction(Vector3f a, Vector3f b, Vector3f c) {
        // Compute the cross product of vectors (b - a) and (c - b)
        double crossProductX = (b.y - a.y) * (c.z - b.z) - (b.z - a.z) * (c.y - b.y);
        double crossProductY = (b.z - a.z) * (c.x - b.x) - (b.x - a.x) * (c.z - b.z);
        double crossProductZ = (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x);

        // Determine the direction based on the sign of the cross product components
        if (Math.abs(crossProductX) < EPSILON && Math.abs(crossProductY) < EPSILON && Math.abs(crossProductZ) < EPSILON) {
            // Collinear
            return 0;
        } else if (crossProductX > 0 || (Math.abs(crossProductX) < EPSILON && crossProductY > 0) || (Math.abs(crossProductX) < EPSILON && Math.abs(crossProductY) < EPSILON && crossProductZ > 0)) {
            // Counterclockwise direction (or non-collinear)
            return 1;
        } else {
            // Clockwise direction
            return -1;
        }
    }

    public static boolean isPointInsideFace(Vector3f v0, Vector3f v1, List<Vector3f> vertices) {
        float dot00, dot01, dot02, dot11, dot12;
        float invDenom;

        Vector3f edge0 = v1.sub(v0);
        Vector3f edge1 = vertices.get(0).sub(v0);
        Vector3f edge2 = vertices.get(1).sub(v0);

        dot00 = edge0.dot(edge0);
        dot01 = edge0.dot(edge1);
        dot02 = edge0.dot(edge2);
        dot11 = edge1.dot(edge1);
        dot12 = edge1.dot(edge2);

        invDenom = 1.0f / (dot00 * dot11 - dot01 * dot01);

        float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
        float v = (dot00 * dot12 - dot01 * dot02) * invDenom;

        return (u >= 0) && (v >= 0) && (u + v <= 1);
    }

    public static Vector3f closestPointBetweenTwoLines(Vector3f line1Start, Vector3f line1End, Vector3f line2Start, Vector3f line2End) {
        // Calculate direction vectors of the two lines
        Vector3f dir1 = line1End.sub(line1Start);
        Vector3f dir2 = line2End.sub(line2Start);

        // Vector between the starting points of the two lines
        Vector3f start1ToStart2 = line2Start.sub(line1Start);

        // Calculate the coefficients of the equations for the lines
        float a = dir1.dot(dir1);
        float b = dir1.dot(dir2);
        float c = dir2.dot(dir2);
        float d = dir1.dot(start1ToStart2);
        float e = dir2.dot(start1ToStart2);

        // Calculate the denominator for finding the closest point
        float denominator = a * c - b * b;

        // Initialize the parameters for the closest points on each line
        float t1, t2;

        // Check if the denominator is close to zero, meaning the lines are nearly parallel
        if (denominator < 0.0001f) {
            // If the lines are nearly parallel, use the starting points of both lines as the closest points
            t1 = 0;
            t2 = e / c;
        } else {
            // Calculate the parameters for the closest points on each line
            t1 = (b * e - c * d) / denominator;
            t2 = (a * e - b * d) / denominator;
        }

        // Calculate the closest points on each line
        Vector3f closestPoint1 = line1Start.add(dir1.mul(t1));
        Vector3f closestPoint2 = line2Start.add(dir2.mul(t2));

        // Calculate and return the midpoint between the two closest points
        return closestPoint1.add(closestPoint2).div(2.0f);
    }


}
