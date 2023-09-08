package collisionDetection.broadPhase;

import collisionDetection.narrowPhase.Shape;
import collisionDetection.primitive.Ray;
import math.Vector3f;

import java.util.Objects;

public class BPBox {
    private Vector3f min; // Min corner of the AABB
    private Vector3f max; // Max corner of the AABB
    private Shape shape;

    public BPBox(Vector3f min, Vector3f max, Shape shape) {
        this.min = min;
        this.max = max;
        this.shape = shape;
    }

    public Vector3f getMax() {
        return max;
    }

    public void setMax(Vector3f max) {
        this.max = max;
    }

    public Shape getShape() {
        return shape;
    }

    public void setShape(Shape shape) {
        this.shape = shape;
    }

    public Vector3f getMin() {
        return min;
    }

    public void setMin(Vector3f min) {
        this.min = min;
    }

    public static boolean isCollide(BPBox box1, BPBox box2) {
        return !(box2.getMin().x > box1.getMax().x || box2.getMax().x < box1.getMin().x ||
                box2.getMin().y > box1.getMax().y || box2.getMax().y < box1.getMin().y ||
                box2.getMin().z > box1.getMax().z || box2.getMax().z < box1.getMin().z);
    }

    public static boolean isCollide(Ray ray, BPBox aabb) {
        // Find the two intersection points on the AABB using the ray's closest point method
        Vector3f p1 = ray.closestPoint(aabb.getMin());
        Vector3f p2 = ray.closestPoint(aabb.getMax());

        // Check if the intersection points are within the AABB's boundaries
        boolean betweenX = (p1.x >= aabb.getMin().x && p1.x <= aabb.getMax().x) || (p2.x >= aabb.getMin().x && p2.x <= aabb.getMax().x);
        boolean betweenY = (p1.y >= aabb.getMin().y && p1.y <= aabb.getMax().y) || (p2.y >= aabb.getMin().y && p2.y <= aabb.getMax().y);
        boolean betweenZ = (p1.z >= aabb.getMin().z && p1.z <= aabb.getMax().z) || (p2.z >= aabb.getMin().z && p2.z <= aabb.getMax().z);

        // If there's an intersection along all axes, the AABB and ray collide
        return betweenX && betweenY && betweenZ;
    }

    @Override
    public String toString() {
        return "BPBox{" +
                "min=" + min +
                ", max=" + max +
                ", shape=" + shape +
                '}';
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        BPBox bpBox = (BPBox) o;
        return Objects.equals(min, bpBox.min) && Objects.equals(max, bpBox.max) && Objects.equals(shape, bpBox.shape);
    }

    @Override
    public int hashCode() {
        return Objects.hash(min, max, shape);
    }
}
