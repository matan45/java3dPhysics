package collisionDetection.primitive;


import collisionDetection.narrowPhase.Shape;
import collisionDetection.narrowPhase.sat.Interval;
import collisionDetection.narrowPhase.sat.SATSupport;
import math.Vector3f;

public class AABB implements Shape, SATSupport {
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
    public boolean isPointInside(Vector3f point) {
        return point.x >= min.x && point.x <= max.x &&
                point.y >= min.y && point.y <= max.y &&
                point.z >= min.z && point.z <= max.z;
    }

    @Override
    public Vector3f closestPoint(Vector3f point) {
        float closestX = Math.max(min.x, Math.min(point.x, max.x));
        float closestY = Math.max(min.y, Math.min(point.y, max.y));
        float closestZ = Math.max(min.z, Math.min(point.z, max.z));

        return new Vector3f(closestX, closestY, closestZ);
    }

    public static boolean isAABBColliding(AABB box1, AABB box2) {
        return !(box2.getMin().x > box1.getMax().x || box2.getMax().x < box1.getMin().x ||
                box2.getMin().y > box1.getMax().y || box2.getMax().y < box1.getMin().y ||
                box2.getMin().z > box1.getMax().z || box2.getMax().z < box1.getMin().z);
    }

    // Helper method to calculate the closest point on a line segment to an AABB
    public Vector3f closestPointOnSegmentToAABB(Vector3f start, Vector3f end) {
        // Calculate the direction of the line segment
        Vector3f segmentDirection = end.sub(start);

        // Calculate the parameter along the segment where the closest point lies
        float t = segmentDirection.dot(getMin().sub(start)) / segmentDirection.lengthSquared();

        // Clamp the parameter to ensure the point is within the segment
        t = Math.max(0, Math.min(1, t));

        // Calculate the closest point on the segment to the AABB
        return start.add(segmentDirection.mul(t));
    }

    @Override
    public Interval getInterval(Vector3f axis) {
        float minProjection = axis.dot(getMin());
        float maxProjection = axis.dot(getMax());

        // Calculate the interval
        float minInterval = Math.min(minProjection, maxProjection);
        float maxInterval = Math.max(minProjection, maxProjection);

        return new Interval(minInterval, maxInterval);
    }

    @Override
    public String toString() {
        return "AABB{" +
                "min=" + min +
                ", max=" + max +
                '}';
    }
}
