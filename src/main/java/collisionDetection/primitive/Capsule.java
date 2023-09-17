package collisionDetection.primitive;


import collisionDetection.narrowPhase.Shape;
import collisionDetection.narrowPhase.gjk.GJKSupport;
import math.Quaternion;
import math.Vector3f;

import java.util.Objects;

public class Capsule implements Shape, GJKSupport {
    private Vector3f start;
    private Vector3f end;
    private float radius;

    public Capsule(Vector3f start, Vector3f end, float radius) {
        this.start = start;
        this.end = end;
        this.radius = radius;
    }

    public Vector3f getStart() {
        return start;
    }

    public void setStart(Vector3f start) {
        this.start = start;
    }

    public Vector3f getEnd() {
        return end;
    }

    public void setEnd(Vector3f end) {
        this.end = end;
    }

    public float getRadius() {
        return radius;
    }

    public void setRadius(float radius) {
        this.radius = radius;
    }

    @Override
    public boolean isPointInside(Vector3f point) {
        // Calculate the capsule axis vector and vectors to the start and end points
        Vector3f axis = end.sub(start);
        Vector3f vecToPoint = point.sub(start);

        // Calculate projection of vecToPoint onto the capsule axis
        float t = vecToPoint.dot(axis) / axis.dot(axis);

        if (t < 0) {
            // Closest point is the start point
            return point.distanceSquared(start) <= radius * radius;
        } else if (t > 1) {
            // Closest point is the end point
            return point.distanceSquared(end) <= radius * radius;
        }

        // Closest point is on the capsule's axis
        Vector3f closestPointOnAxis = start.add(axis.mul(t));
        return point.distanceSquared(closestPointOnAxis) <= radius * radius;

    }

    @Override
    public Vector3f closestPoint(Vector3f point) {
        Vector3f axis = end.sub(start);
        Vector3f vecToPoint = point.sub(start);

        float t = vecToPoint.dot(axis) / axis.dot(axis);
        t = Math.max(0, Math.min(1, t)); // Clamp t to [0, 1]

        Vector3f closestPointOnAxis = start.add(axis.mul(t));

        return closestPointOnAxis.add(vecToPoint.sub(closestPointOnAxis).normalize().mul(radius));
    }

    @Override
    public void translate(Vector3f position) {
        start.set(start.add(position));
        end.set(end.add(position));
    }

    @Override
    public void scale(Vector3f scale) {
        // Scale the radius
        radius *= Math.max(scale.x, Math.max(scale.y, scale.z));
    }

    @Override
    public void rotate(Quaternion rotate) {
        start.set(start.rotate(rotate));
        end.set(end.rotate(rotate));
    }

    @Override
    public Vector3f support(Vector3f direction) {

        // For simplicity, let's assume the support point is at the center of the capsule.
        Vector3f center = start.add(end).div(2.0f);

        // Now, move the center in the direction specified by the input vector.
        return center.add(direction.normalize().mul(radius));
    }

    @Override
    public String toString() {
        return "Capsule{" +
                "start=" + start +
                ", end=" + end +
                ", radius=" + radius +
                '}';
    }

    @Override
    public int hashCode() {
        return Objects.hash(start, end, radius);
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Capsule capsule = (Capsule) o;
        return Float.compare(capsule.radius, radius) == 0 && Objects.equals(start, capsule.start) && Objects.equals(end, capsule.end);
    }
}
