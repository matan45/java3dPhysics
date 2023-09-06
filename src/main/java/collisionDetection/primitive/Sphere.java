package collisionDetection.primitive;

import collisionDetection.narrowPhase.Shape;
import collisionDetection.narrowPhase.gjk.GJKSupport;
import math.Vector3f;

import java.util.Objects;

public class Sphere implements Shape, GJKSupport {
    private Vector3f center; // Center of the sphere
    private float radius; // Radius of the sphere

    public Sphere(Vector3f center, float radius) {
        this.center = center;
        this.radius = radius;
    }

    public Vector3f getCenter() {
        return center;
    }

    public void setCenter(Vector3f center) {
        this.center = center;
    }

    public float getRadius() {
        return radius;
    }

    public void setRadius(float radius) {
        this.radius = radius;
    }

    @Override
    public boolean isPointInside(Vector3f point) {
        float distanceSquared = center.distanceSquared(point);
        return distanceSquared <= radius * radius;
    }

    @Override
    public Vector3f closestPoint(Vector3f point) {
        Vector3f direction = point.sub(center);
        direction = direction.normalize();
        return center.add(direction.mul(radius));
    }

    @Override
    public String toString() {
        return "Sphere{" +
                "center=" + center +
                ", radius=" + radius +
                '}';
    }

    @Override
    public int hashCode() {
        return Objects.hash(center, radius);
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Sphere sphere = (Sphere) o;
        return Float.compare(sphere.radius, radius) == 0 && Objects.equals(center, sphere.center);
    }

    @Override
    public Vector3f support(Vector3f direction) {
        return center.add(direction.normalize().mul(radius));
    }
}
