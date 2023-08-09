package collisionDetection.primitive;

import org.joml.Vector3f;

public class Capsule {
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
