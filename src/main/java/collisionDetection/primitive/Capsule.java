package collisionDetection.primitive;


import collisionDetection.narrowPhase.Shape;
import collisionDetection.narrowPhase.gjk.GJKSupport;
import collisionDetection.narrowPhase.sat.Interval;
import collisionDetection.narrowPhase.sat.SATSupport;
import math.Vector3f;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

public class Capsule implements Shape, GJKSupport, SATSupport {
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
    public Vector3f support(Vector3f direction) {
        Vector3f capsuleAxis = end.sub(start); // Compute the capsule's axis

        // Project the direction onto the capsule's axis
        float projection = direction.dot(capsuleAxis);

        // Calculate the endpoints of the capsule
        Vector3f capsuleStart = start.add(capsuleAxis.mul(projection));
        Vector3f capsuleEnd = end.add(capsuleAxis.mul(projection));

        // Calculate the center of the capsule
        Vector3f capsuleCenter = capsuleStart.add(capsuleEnd).div(2.0f);

        // Calculate the support point by moving from the center along the direction
        return capsuleCenter.add(direction.normalize().mul(radius));
    }

    @Override
    public List<Vector3f> getAxis() {
        List<Vector3f> axes = new ArrayList<>();

        // Axis along the capsule's central line
        Vector3f capsuleAxis = end.sub(start).normalize();
        axes.add(capsuleAxis);

        // Perpendicular axis (choose any two perpendicular vectors)
        Vector3f arbitraryVector1 = Vector3f.XAxis;
        Vector3f perpendicularAxis = capsuleAxis.cross(arbitraryVector1);

        if (perpendicularAxis.lengthSquared() < 0.0001) {
            // If the cross product is too small, choose a different arbitrary vector
            arbitraryVector1 = Vector3f.YAxis;
            perpendicularAxis = capsuleAxis.cross(arbitraryVector1);
        }
        axes.add(perpendicularAxis.normalize());

        return axes;
    }

    @Override
    public Interval getInterval(Vector3f axis) {
        // Calculate the projections of the capsule's endpoints onto the axis
        float projectionStart = start.dot(axis);
        float projectionEnd = end.dot(axis);

        // Calculate the projection of the center of the capsule
        Vector3f capsuleCenter = new Vector3f(
                (start.x + end.x) / 2,
                (start.y + end.y) / 2,
                (start.z + end.z) / 2
        );
        float projectionCenter = capsuleCenter.dot(axis);

        // Calculate the half-length of the capsule along the axis
        float halfLength = (projectionEnd - projectionStart) / 2;

        // Calculate the interval min and max
        float min = projectionCenter - halfLength - radius;
        float max = projectionCenter + halfLength + radius;

        return new Interval(min, max);
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
