package collisionDetection.primitive;

import collisionDetection.narrowPhase.Shape;
import math.Vector3f;

import static math.Const.EPSILON;

public class Line implements Shape {
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

    public static boolean isLineColliding(Line line1, Line line2) {
        // Get the direction vectors of both lines.
        Vector3f dir1 = line1.getEnd().sub(line1.getStart());
        Vector3f dir2 = line2.getEnd().sub(line2.getStart());

        // Calculate the determinant of the direction vectors.
        float determinant = dir1.x * dir2.y - dir1.y * dir2.x;

        // If the determinant is close to zero, the lines are parallel and may not intersect.
        if (Math.abs(determinant) < EPSILON) {
            return false;
        }

        // Calculate parameters for the lines' parametric equations.
        Vector3f toStart2 = line2.getStart().sub(line1.getStart());
        float t1 = (toStart2.x * dir2.y - toStart2.y * dir2.x) / determinant;
        float t2 = (toStart2.x * dir1.y - toStart2.y * dir1.x) / determinant;

        // Check if the intersection points are within the valid range [0, 1] for both lines.
        return t1 >= 0 && t1 <= 1 && t2 >= 0 && t2 <= 1;  // Lines intersect within their segments.
    }



    @Override
    public String toString() {
        return "Line{" +
                "start=" + start +
                ", end=" + end +
                '}';
    }


}
