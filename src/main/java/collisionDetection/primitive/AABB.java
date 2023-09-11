package collisionDetection.primitive;


import collisionDetection.narrowPhase.Shape;
import collisionDetection.narrowPhase.gjk.GJKSupport;
import collisionDetection.narrowPhase.sat.Interval;
import collisionDetection.narrowPhase.sat.SATSupport;
import math.Vector3f;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

public class AABB implements Shape, SATSupport, GJKSupport {
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

    public Vector3f getCenter() {
        float centerX = (min.x + max.x) / 2.0f;
        float centerY = (min.y + max.y) / 2.0f;
        float centerZ = (min.z + max.z) / 2.0f;

        return new Vector3f(centerX, centerY, centerZ);
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
    public List<Vector3f> getAxis() {
        return List.of(new Vector3f(1, 0, 0),
                new Vector3f(0, 1, 0),
                new Vector3f(0, 0, 1));
    }

    @Override
    public List<Vector3f> getVertices() {
        List<Vector3f> vertices = new ArrayList<>(8);

        // Define the eight vertices of the AABB
        vertices.add(new Vector3f(min.x, min.y, min.z));
        vertices.add(new Vector3f(max.x, min.y, min.z));
        vertices.add(new Vector3f(min.x, max.y, min.z));
        vertices.add(new Vector3f(max.x, max.y, min.z));
        vertices.add(new Vector3f(min.x, min.y, max.z));
        vertices.add(new Vector3f(max.x, min.y, max.z));
        vertices.add(new Vector3f(min.x, max.y, max.z));
        vertices.add(new Vector3f(max.x, max.y, max.z));

        return vertices;
    }

    @Override
    public Vector3f support(Vector3f direction) {
        Vector3f result = new Vector3f();

        result.x = (direction.x >= 0) ? max.x : min.x;
        result.y = (direction.y >= 0) ? max.y : min.y;
        result.z = (direction.z >= 0) ? max.z : min.z;

        return result;
    }

    public static boolean isCollide(AABB box1, AABB box2) {
        return !(box2.getMin().x > box1.getMax().x || box2.getMax().x < box1.getMin().x ||
                box2.getMin().y > box1.getMax().y || box2.getMax().y < box1.getMin().y ||
                box2.getMin().z > box1.getMax().z || box2.getMax().z < box1.getMin().z);
    }

    @Override
    public String toString() {
        return "AABB{" +
                "min=" + min +
                ", max=" + max +
                '}';
    }

    @Override
    public int hashCode() {
        return Objects.hash(min, max);
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        AABB aabb = (AABB) o;
        return Objects.equals(min, aabb.min) && Objects.equals(max, aabb.max);
    }
}
