package collisionDetection.primitive;

import collisionDetection.narrowPhase.Shape;
import collisionDetection.narrowPhase.gjk.GJKSupport;
import collisionDetection.narrowPhase.sat.Interval;
import collisionDetection.narrowPhase.sat.SATSupport;
import math.Vector3f;

import java.util.ArrayList;
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
        // Check if the point is within the bounding box of the line segment
        return !(point.x < Math.min(start.x, end.x)) && !(point.x > Math.max(start.x, end.x)) &&
                !(point.y < Math.min(start.y, end.y)) && !(point.y > Math.max(start.y, end.y)) &&
                !(point.z < Math.min(start.z, end.z)) && !(point.z > Math.max(start.z, end.z));
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
        // Calculate the dot products of the direction vector with the start and end points
        float dotStart = start.dot(direction);
        float dotEnd = end.dot(direction);

        // Determine which endpoint is farthest in the given direction
        if (dotStart >= dotEnd) {
            return start; // Start point is the support point
        }
        return end; // End point is the support point

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

    @Override
    public Interval getInterval(Vector3f axis) {
        // Project the line onto the axis
        float dotStart = start.dot(axis);
        float dotEnd = end.dot(axis);

        // Calculate the minimum and maximum values along the axis
        float min = Math.min(dotStart, dotEnd);
        float max = Math.max(dotStart, dotEnd);

        return new Interval(min, max);
    }

    @Override
    public List<Vector3f> getAxis() {
        // For a line segment, there are three unique axes: the direction of the line,
        // the perpendicular axis, and the z-axis.
        Vector3f lineDirection = end.sub(start).normalize();

        // Create two arbitrary vectors that are not collinear with the line segment.
        // Calculate two perpendicular axes using the cross product.
        Vector3f perpendicularAxis1 = lineDirection.cross(Vector3f.XAxis).normalize();
        Vector3f perpendicularAxis2 = lineDirection.cross(Vector3f.YAxis).normalize();
        Vector3f perpendicularAxis3 = lineDirection.cross(Vector3f.ZAxis).normalize();

        List<Vector3f> axes = new ArrayList<>();
        axes.add(lineDirection);
        axes.add(perpendicularAxis1);
        axes.add(perpendicularAxis2);
        axes.add(perpendicularAxis3);

        return axes;
    }

    @Override
    public List<Vector3f> getVertices() {
        return List.of(start, end);
    }
}
