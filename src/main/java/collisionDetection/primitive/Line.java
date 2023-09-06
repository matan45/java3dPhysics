package collisionDetection.primitive;

import collisionDetection.narrowPhase.Shape;
import collisionDetection.narrowPhase.gjk.GJKSupport;
import collisionDetection.narrowPhase.sat.Interval;
import collisionDetection.narrowPhase.sat.SATSupport;
import math.Vector3f;

import java.util.List;
import java.util.Objects;

public class Line implements Shape, GJKSupport, SATSupport {
    private Vector3f start;
    private Vector3f end;

    public Line(Vector3f start, Vector3f end) {
        this.start = start;
        this.end = end;
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

    @Override
    public boolean isPointInside(Vector3f point) {
        // To determine if a point is inside the line, you can check if the point is collinear
        // with the start and end points. If it is, then it lies on the line.
        Vector3f direction = end.sub(start);
        Vector3f toPoint = point.sub(start);

        float dotProduct = direction.dot(toPoint);

        // If the dot product is between 0 and the squared length of the direction vector,
        // then the point is inside the line segment.
        return dotProduct >= 0 && dotProduct <= direction.lengthSquared();
    }

    @Override
    public Vector3f closestPoint(Vector3f point) {
        // To find the closest point on the line to the given point, you can project
        // the vector from the start point to the given point onto the line's direction.
        Vector3f direction = end.sub(start);
        Vector3f toPoint = point.sub(start);

        float t = toPoint.dot(direction) / direction.lengthSquared();

        // Ensure t is within the valid range [0, 1] for a point on the line segment.
        t = Math.max(0, Math.min(1, t));

        // Calculate the closest point by adding the scaled direction to the start point.
        return start.add(direction.mul(t));
    }

    @Override
    public Vector3f support(Vector3f direction) {
        if (start.dot(direction) > end.dot(direction))
            return start;
        return end;
    }

    @Override
    public Interval getInterval(Vector3f axis) {
        // Calculate the projections of the line segment's endpoints onto the axis
        float projectionStart = start.dot(axis);
        float projectionEnd = end.dot(axis);

        // Calculate the interval min and max
        float min = Math.min(projectionStart, projectionEnd);
        float max = Math.max(projectionStart, projectionEnd);

        return new Interval(min, max);
    }

    @Override
    public List<Vector3f> getAxis() {
        // Calculate the axis along the line segment
        return List.of(end.sub(start).normalize());
    }

    @Override
    public String toString() {
        return "Line{" +
                "start=" + start +
                ", end=" + end +
                '}';
    }

    @Override
    public int hashCode() {
        return Objects.hash(start, end);
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Line line = (Line) o;
        return Objects.equals(start, line.start) && Objects.equals(end, line.end);
    }
}
