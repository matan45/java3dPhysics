package collisionDetection.primitive;

import collisionDetection.narrowPhase.Shape;
import collisionDetection.narrowPhase.gjk.GJKSupport;
import collisionDetection.narrowPhase.sat.Interval;
import collisionDetection.narrowPhase.sat.SATSupport;
import math.Vector3f;

import java.util.List;
import java.util.Objects;

import static math.Const.EPSILON;

public class Plane implements Shape, GJKSupport, SATSupport {
    private Vector3f normal; // Normal vector of the plane
    private float distance;  // Distance from the origin to the plane along the normal

    public Plane(Vector3f normal, float distance) {
        this.normal = normal.normalize(); // Normalize the normal vector
        this.distance = distance;
    }

    public boolean isPointInFront(Vector3f point) {
        return normal.dot(point) + distance > 0;
    }

    public Vector3f getNormal() {
        return normal;
    }

    public void setNormal(Vector3f normal) {
        this.normal = normal;
    }

    public float getDistance() {
        return distance;
    }

    public void setDistance(float distance) {
        this.distance = distance;
    }

    @Override
    public boolean isPointInside(Vector3f point) {
        // Calculate the signed distance from the point to the plane
        float signedDistance = point.dot(normal) - distance;

        // If the signed distance is very close to zero, consider the point to be on the plane
        return Math.abs(signedDistance) < EPSILON;
    }

    @Override
    public Vector3f closestPoint(Vector3f point) {
        float dot = point.dot(normal);
        float dis = dot - distance;
        return point.sub(normal.mul(dis));
    }

    @Override
    public Vector3f support(Vector3f direction) {
        // Calculate the dot product of the direction vector and the plane's normal
        float dotProduct = normal.dot(direction);

        // Use the dot product to calculate the furthest point in the given direction
        return normal.mul(distance / dotProduct);
    }

    @Override
    public String toString() {
        return "Plane{" +
                "normal=" + normal +
                ", distance=" + distance +
                '}';
    }

    @Override
    public int hashCode() {
        return Objects.hash(normal, distance);
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Plane plane = (Plane) o;
        return Float.compare(plane.distance, distance) == 0 && Objects.equals(normal, plane.normal);
    }

    @Override
    public Interval getInterval(Vector3f axis) {
        // Project the plane onto the given axis
        float projection = normal.dot(axis);

        // Calculate the half-extent of the projection
        float halfExtent = Math.abs(projection) * distance;

        // Calculate the minimum and maximum values of the interval
        float minInterval = -halfExtent;
        float maxInterval = halfExtent;

        // Create and return an interval
        return new Interval(minInterval, maxInterval);
    }

    @Override
    public List<Vector3f> getAxis() {
        return List.of(normal);
    }
}
