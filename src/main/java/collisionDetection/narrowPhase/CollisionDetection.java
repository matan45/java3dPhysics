package collisionDetection.narrowPhase;

import collisionDetection.narrowPhase.sat.Interval;
import collisionDetection.primitive.*;
import collisionDetection.primitive.terrain.TerrainShape;
import math.Vector3f;

import java.util.List;

import static math.Const.EPSILON;

public class CollisionDetection {

    public static boolean isCollide(AABB aabb, OBB obb) {
        // Combine axes from OBB and aabb
        Vector3f[] test = new Vector3f[18];

        test[0] = new Vector3f(1, 0, 0);
        test[1] = new Vector3f(0, 1, 0);
        test[2] = new Vector3f(0, 0, 1);
        test[3] = obb.getAxis().get(0);
        test[4] = obb.getAxis().get(1);
        test[5] = obb.getAxis().get(2);

        for (int i = 0; i < 3; ++i) {
            test[6 + i * 3] = test[i].cross(test[3]);
            test[6 + i * 3 + 1] = test[i].cross(test[4]);
            test[6 + i * 3 + 2] = test[i].cross(test[5]);
        }

        // Include edge normals of both OBBs
        for (int i = 0; i < 3; ++i) {
            test[12 + i] = obb.getEdge(i).normalize();
            test[15 + i] = obb.getEdge(i).normalize();
        }

        // Check for separation along each axis
        for (Vector3f axis : test) {
            if (isAxisSeparating(axis, aabb, obb)) {
                return false; // No collision along this axis
            }
        }

        return true; // No separation along any axis, collision detected
    }

    private static boolean isAxisSeparating(Vector3f axis, AABB aabb, OBB obb) {
        // Project the OBBs onto the axis
        Interval projection1 = aabb.getInterval(axis);
        Interval projection2 = obb.getInterval(axis);

        // Check for separation between the intervals
        return projection1.getMax() < projection2.getMin() || projection2.getMax() < projection1.getMin();
    }

    public static boolean isCollide(Triangle triangle, AABB aabb) {

        // Check if any of the triangle's vertices are inside the AABB
        if (aabb.isPointInside(triangle.getVertex1()) ||
                aabb.isPointInside(triangle.getVertex2()) ||
                aabb.isPointInside(triangle.getVertex3())) {
            return true;
        }

        // Check if the triangle intersects any of the AABB's edges
        return isTriangleEdgeIntersectingAABB(triangle.getVertex1(), triangle.getVertex2(), aabb) ||
                isTriangleEdgeIntersectingAABB(triangle.getVertex2(), triangle.getVertex3(), aabb) ||
                isTriangleEdgeIntersectingAABB(triangle.getVertex1(), triangle.getVertex3(), aabb);
    }

    private static boolean isTriangleEdgeIntersectingAABB(Vector3f vertex1, Vector3f vertex2, AABB aabb) {
        // Check if either vertex of the edge is inside the AABB
        if (aabb.isPointInside(vertex1) || aabb.isPointInside(vertex2)) {
            return true;
        }

        // Calculate the direction and length of the edge
        Vector3f edgeDirection = vertex2.sub(vertex1);
        float edgeLength = edgeDirection.length();
        edgeDirection.normalize();

        // Calculate the minimum and maximum values of t for intersection
        float tMin = 0.0f;
        float tMax = edgeLength;

        // Perform intersection tests with each axis of the AABB
        for (int i = 0; i < 3; i++) {
            if (Math.abs(edgeDirection.get(i)) < EPSILON) {
                // Edge is parallel to the AABB face, so no intersection possible
                if (vertex1.get(i) < aabb.getMin().get(i) || vertex1.get(i) > aabb.getMax().get(i)) {
                    return false;
                }
            } else {
                float t1 = (aabb.getMin().get(i) - vertex1.get(i)) / edgeDirection.get(i);
                float t2 = (aabb.getMax().get(i) - vertex1.get(i)) / edgeDirection.get(i);

                if (t1 > t2) {
                    float temp = t1;
                    t1 = t2;
                    t2 = temp;
                }

                if (t1 > tMin) {
                    tMin = t1;
                }

                if (t2 < tMax) {
                    tMax = t2;
                }

                if (tMin > tMax) {
                    return false;
                }
            }
        }

        // Check if the intersection occurred within the edge length
        return tMin <= edgeLength && tMax >= 0;
    }

    public static boolean isCollide(ConvexPolyhedron convexPolyhedron1, ConvexPolyhedron convexPolyhedron2) {
        List<Vector3f> vertices1 = convexPolyhedron1.getVertices();
        List<Vector3f> vertices2 = convexPolyhedron2.getVertices();

        // Loop through all the edges of both polyhedra
        for (int i = 0; i < vertices1.size(); i++) {
            Vector3f edgeStart = vertices1.get(i);
            Vector3f edgeEnd = vertices1.get((i + 1) % vertices1.size());

            Vector3f axis = edgeEnd.sub(edgeStart).normalize();

            if (isAxisSeparating(axis, convexPolyhedron1, convexPolyhedron2)) {
                return false;
            }
        }

        for (int i = 0; i < vertices2.size(); i++) {
            Vector3f edgeStart = vertices2.get(i);
            Vector3f edgeEnd = vertices2.get((i + 1) % vertices2.size());

            Vector3f axis = edgeEnd.sub(edgeStart).normalize();

            if (isAxisSeparating(axis, convexPolyhedron1, convexPolyhedron2)) {
                return false;
            }
        }

        return true;
    }

    private static boolean isAxisSeparating(Vector3f axis, ConvexPolyhedron convexPolyhedron1, ConvexPolyhedron convexPolyhedron2) {
        // Project the OBBs onto the axis
        Interval projection1 = convexPolyhedron1.getInterval(axis);
        Interval projection2 = convexPolyhedron2.getInterval(axis);

        // Check for separation between the intervals
        return projection1.getMax() < projection2.getMin() || projection2.getMax() < projection1.getMin();
    }

    public static boolean isCollide(OBB obb1, OBB obb2) {
        // Combine axes from both OBBs
        Vector3f[] test = new Vector3f[18];

        test[0] = obb1.getAxis().get(0);
        test[1] = obb1.getAxis().get(1);
        test[2] = obb1.getAxis().get(2);
        test[3] = obb2.getAxis().get(0);
        test[4] = obb2.getAxis().get(1);
        test[5] = obb2.getAxis().get(2);

        for (int i = 0; i < 3; ++i) {
            test[6 + i * 3] = test[i].cross(test[3]);
            test[6 + i * 3 + 1] = test[i].cross(test[4]);
            test[6 + i * 3 + 2] = test[i].cross(test[5]);
        }

        // Include edge normals of both OBBs
        for (int i = 0; i < 3; ++i) {
            test[12 + i] = obb1.getEdge(i).normalize();
            test[15 + i] = obb2.getEdge(i).normalize();
        }

        // Check for separation along each axis
        for (Vector3f axis : test) {
            if (isAxisSeparating(axis, obb1, obb2)) {
                return false; // No collision along this axis
            }
        }

        return true; // No separation along any axis, collision detected
    }

    private static boolean isAxisSeparating(Vector3f axis, OBB obb1, OBB obb2) {
        // Project the OBBs onto the axis
        Interval projection1 = obb1.getInterval(axis);
        Interval projection2 = obb2.getInterval(axis);

        // Check for separation between the intervals
        return projection1.getMax() < projection2.getMin() || projection2.getMax() < projection1.getMin();
    }

    public static boolean isCollide(Triangle triangle1, Triangle triangle2) {
        // Axes to test
        Vector3f[] axes = {
                triangle1.calculateFaceNormal(),
                triangle2.calculateFaceNormal(),
                triangle1.getEdge1().normalize(),
                triangle1.getEdge2().normalize(),
                triangle1.getEdge3().normalize(),
                triangle2.getEdge1().normalize(),
                triangle2.getEdge2().normalize(),
                triangle2.getEdge3().normalize()
        };

        for (Vector3f axis : axes) {
            if (isSeparatingAxis(axis, triangle1, triangle2)) {
                return false; // No collision along this axis
            }
        }

        return true; // No separation along any axis, collision detected
    }

    private static boolean isSeparatingAxis(Vector3f axis, Triangle triangle1, Triangle triangle2) {
        Interval interval1 = triangle1.getInterval(axis);
        Interval interval2 = triangle2.getInterval(axis);

        return interval1.getMax() < interval2.getMin() || interval2.getMax() < interval1.getMin();
    }

    public static boolean isCollide(AABB box1, AABB box2) {
        return !(box2.getMin().x > box1.getMax().x || box2.getMax().x < box1.getMin().x ||
                box2.getMin().y > box1.getMax().y || box2.getMax().y < box1.getMin().y ||
                box2.getMin().z > box1.getMax().z || box2.getMax().z < box1.getMin().z);
    }

}
