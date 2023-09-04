package collisionDetection.util;

import collisionDetection.narrowPhase.gjk.GJKSupport;
import collisionDetection.primitive.*;
import math.Vector3f;

public class CollisionUtil {

    public static Vector3f support(GJKSupport shape1, GJKSupport shape2, Vector3f direction) {
        Vector3f pointA = shape1.support(direction);
        Vector3f pointB = shape2.support(direction.negate());
        return pointA.sub(pointB);
    }

    // Helper method to calculate the closest point on a line segment to an AABB
    public static Vector3f closestPointOnSegmentToAABB(AABB aabb, Line line) {
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

   public static boolean isCylinderSideCollidingWithTriangle(Cylinder cylinder, Triangle triangle) {
        // Calculate the triangle's normal
        Vector3f triangleNormal = triangle.calculateTriangleNormal();

        // Calculate the projection of the cylinder onto the triangle's plane
        Vector3f cylinderProjectionCenter = new Vector3f(cylinder.getCenter().x, triangle.getVertex1().y, cylinder.getCenter().z);

        // Project the cylinder's top and bottom circles onto the triangle's plane
        Vector3f projectedTopCenter = cylinderProjectionCenter.add(triangleNormal.mul(cylinder.getRadius()));
        Vector3f projectedBottomCenter = cylinderProjectionCenter.sub(triangleNormal.mul(cylinder.getRadius()));

        // Check if the projected circles intersect the triangle
        return isCircleIntersectingTriangle(projectedTopCenter, cylinder.getRadius(), triangle) ||
                isCircleIntersectingTriangle(projectedBottomCenter, cylinder.getRadius(), triangle);
    }

    private static boolean isCircleIntersectingTriangle(Vector3f circleCenter, float radius, Triangle triangle) {
        // Check if any vertex of the triangle is inside the circle
        if (isPointInsideCircle(circleCenter, radius, triangle.getVertex1()) ||
                isPointInsideCircle(circleCenter, radius, triangle.getVertex2()) ||
                isPointInsideCircle(circleCenter, radius, triangle.getVertex3())) {
            return true;
        }

        // Check if any triangle edge intersects the circle
        return isSegmentIntersectingCircle(triangle.getVertex1(), triangle.getVertex2(), circleCenter, radius) ||
                isSegmentIntersectingCircle(triangle.getVertex2(), triangle.getVertex3(), circleCenter, radius) ||
                isSegmentIntersectingCircle(triangle.getVertex3(), triangle.getVertex1(), circleCenter, radius);
    }

    private static boolean isPointInsideCircle(Vector3f circleCenter, float radius, Vector3f point) {
        float distanceSquared = point.sub(circleCenter).lengthSquared();
        return distanceSquared <= radius * radius;
    }

    private static boolean isSegmentIntersectingCircle(Vector3f startPoint, Vector3f endPoint, Vector3f circleCenter, float radius) {
        // Perform intersection test between a line segment and a circle
        // This is a simplified version and may not handle all cases
        // A more accurate algorithm like the Bresenham algorithm should be used

        // Calculate the closest point on the segment to the circle center
        Vector3f segmentDirection = endPoint.sub(startPoint);
        Vector3f toCircleCenter = circleCenter.sub(startPoint);
        float t = toCircleCenter.dot(segmentDirection) / segmentDirection.lengthSquared();
        t = Math.max(0, Math.min(1, t)); // Clamp t to the [0, 1] interval
        Vector3f closestPoint = startPoint.add(segmentDirection.mul(t));

        // Check if the closest point is within the circle's radius
        return isPointInsideCircle(circleCenter, radius, closestPoint);
    }

    public static float calculateDistanceToPlane(Vector3f point, Vector3f normal, float constant) {
        return normal.dot(point) + constant / normal.length();
    }

    public static boolean isPointCollidingWithCylinderCap(Vector3f point, Cylinder cylinder) {
        // Calculate the distance between the point and the cylinder's center in the XZ plane
        float distanceXZ = (float) Math.sqrt((point.x - cylinder.getCenter().x) * (point.x - cylinder.getCenter().x) +
                (point.z - cylinder.getCenter().z) * (point.z - cylinder.getCenter().z));

        // Check if the point is within the cylinder's cap radius and its Y coordinate is within the cap's height
        return distanceXZ <= cylinder.getRadius() && Math.abs(point.y - cylinder.getCenter().y) <= cylinder.getHeight() / 2.0f;
    }

    public static boolean checkEdgeEdgeCollision(Vector3f A, Vector3f B, Vector3f C, Vector3f D) {
        Vector3f AB = B.sub(A);
        Vector3f AC = C.sub(A);
        Vector3f AD = D.sub(A);

        Vector3f normalABCD = AB.cross(AC);

        // Check if the cross product of AB and AC points in the same direction as AD.
        if (normalABCD.dot(AD) > 0) {
            // Check if point D is on the opposite side of ABC as A is.
            Vector3f BC = C.sub(B);
            Vector3f BD = D.sub(B);
            Vector3f normalBCDA = BC.cross(BD);

            // The edges intersect.
            return normalBCDA.dot(AC) > 0 && normalBCDA.dot(AB) > 0;
        }

        return false;
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

}
