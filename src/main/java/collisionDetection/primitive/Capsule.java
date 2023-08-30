package collisionDetection.primitive;


import collisionDetection.narrowPhase.Shape;
import math.Vector3f;

public class Capsule implements Shape {
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

    public static boolean isCapsuleColliding(Capsule capsule1, Capsule capsule2) {
        float radiusSum = capsule1.getRadius() + capsule2.getRadius();

        // Calculate the squared distance between the capsules' start points
        float distanceSquared = capsule1.getStart().distanceSquared(capsule2.getStart());

        // Check if the distance is less than the sum of the radii
        if (distanceSquared <= (radiusSum * radiusSum)) {
            return true;
        }

        // Calculate the squared distance between the capsules' start points and the end points of each capsule
        float distanceSquaredToC1Start = capsule1.getStart().distanceSquared(capsule2.getEnd());
        float distanceSquaredToC1End = capsule1.getEnd().distanceSquared(capsule2.getStart());
        float distanceSquaredToC2Start = capsule1.getStart().distanceSquared(capsule2.getEnd());
        float distanceSquaredToC2End = capsule1.getEnd().distanceSquared(capsule2.getEnd());

        // Check if any of the distances are less than the sum of the radii
        return distanceSquaredToC1Start <= (radiusSum * radiusSum) ||
                distanceSquaredToC1End <= (radiusSum * radiusSum) ||
                distanceSquaredToC2Start <= (radiusSum * radiusSum) ||
                distanceSquaredToC2End <= (radiusSum * radiusSum);
    }


    @Override
    public String toString() {
        return "Capsule{" +
                "start=" + start +
                ", end=" + end +
                ", radius=" + radius +
                '}';
    }

}
