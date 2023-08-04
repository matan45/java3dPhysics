package collisionDetection.primitive;

import org.joml.Vector3f;

public class AABB {
    private Vector3f min; // Min corner of the AABB
    private Vector3f max; // Max corner of the AABB

    public AABB(Vector3f min, Vector3f max) {
        this.min = min;
        this.max = max;
    }

    public Vector3f getMin() {
        return min;
    }

    public void setMin(Vector3f min) {
        this.min = min;
    }

    public Vector3f getMax() {
        return max;
    }

    public void setMax(Vector3f max) {
        this.max = max;
    }

    @Override
    public String toString() {
        return "AABB{" +
                "min=" + min +
                ", max=" + max +
                '}';
    }

    public static CollisionResult isAABBColliding(AABB box1, AABB box2) {
        if (box1.getMax().x < box2.getMin().x || box1.getMin().x > box2.getMax().x ||
                box1.getMax().y < box2.getMin().y || box1.getMin().y > box2.getMax().y ||
                box1.getMax().z < box2.getMin().z || box1.getMin().z > box2.getMax().z) {
            return closestPointPair(box1, box2, false);
        }

        // If there is no separation along any axis, the AABBs are colliding
        return closestPointPair(box1, box2, true);
    }

    private static double distance(Vector3f p1, Vector3f p2) {
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;
        double dz = p1.z - p2.z;
        return Math.sqrt(dx * dx + dy * dy + dz * dz);
    }

    // Method to find the closest point pair and their distance between two AABBs
    private static CollisionResult closestPointPair(AABB aabb1, AABB aabb2, boolean isCollide) {
        double minDistance = Double.MAX_VALUE;
        Vector3f closestPoint1 = null;
        Vector3f closestPoint2 = null;

        // Calculate the distance between all possible pairs of vertices
        for (int i = 0; i < 8; i++) {
            Vector3f vertex1 = i < 4 ? aabb1.getMin() : aabb1.getMax();
            Vector3f vertex2 = ((i & 1) == 0) ? aabb2.getMin() : aabb2.getMax();
            vertex2 = ((i & 2) == 0) ? new Vector3f(vertex2.x, vertex2.y, vertex1.z) : new Vector3f(vertex1.x, vertex1.y, vertex2.z);

            double distance = distance(vertex1, vertex2);
            if (distance < minDistance) {
                minDistance = distance;
                closestPoint1 = vertex1;
                closestPoint2 = vertex2;
            }
        }

        return new CollisionResult(isCollide, minDistance, closestPoint1, closestPoint2);
    }
}
