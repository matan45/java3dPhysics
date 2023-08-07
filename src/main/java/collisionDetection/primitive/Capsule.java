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

    public static CollisionResult isCapsuleColliding(Capsule capsule1, Capsule capsule2) {
        // Calculate the squared sum of the radii for collision check
        double sumRadiiSquared = (capsule1.radius + capsule2.radius) * (capsule1.radius + capsule2.radius);

        // Calculate vectors between the two capsule endpoints
        Vector3f d1 = capsule2.start.sub(capsule1.start);
        Vector3f d2 = capsule2.start.sub(capsule1.end);
        Vector3f d3 = capsule2.end.sub(capsule1.start);
        Vector3f d4 = capsule2.end.sub(capsule1.end);

        // Calculate squared distances between the endpoints and the capsules
        float dist1Squared = d1.dot(d1);
        float dist2Squared = d2.dot(d2);
        float dist3Squared = d3.dot(d3);
        float dist4Squared = d4.dot(d4);

        // Calculate vectors between capsule directions and cross-products
        Vector3f dir1 = capsule1.end.sub(capsule1.start);
        Vector3f dir2 = capsule2.end.sub(capsule2.start);

        Vector3f cross1 = dir1.cross(d1);
        Vector3f cross2 = dir1.cross(d2);
        Vector3f cross3 = dir2.cross(d3);
        Vector3f cross4 = dir2.cross(d4);

        // Calculate squared distances between capsule directions and cross-products
        float cross1Squared = cross1.dot(cross1);
        float cross2Squared = cross2.dot(cross2);
        float cross3Squared = cross3.dot(cross3);
        float cross4Squared = cross4.dot(cross4);

        float len1Squared = dir1.dot(dir1);

        // Check if any endpoint is inside the other capsule
        if (dist1Squared < sumRadiiSquared || dist2Squared < sumRadiiSquared ||
                dist3Squared < sumRadiiSquared || dist4Squared < sumRadiiSquared) {
            return collisionResultDetails(dist1Squared, dist2Squared, dist3Squared, dist4Squared,
                    d1, d2, dir1, dir2,
                    capsule1, capsule2, len1Squared, true);
        }

        if (cross1Squared < sumRadiiSquared && cross2Squared < sumRadiiSquared &&
                cross3Squared < sumRadiiSquared && cross4Squared < sumRadiiSquared) {
            return collisionResultDetails(dist1Squared, dist2Squared, dist3Squared, dist4Squared,
                    d1, d2, dir1, dir2,
                    capsule1, capsule2, len1Squared, true);

        }

        return collisionResultDetails(dist1Squared, dist2Squared, dist3Squared, dist4Squared,
                d1, d2, dir1, dir2,
                capsule1, capsule2, len1Squared, false);

    }

    private static CollisionResult collisionResultDetails(float dist1Squared, float dist2Squared,
                                                          float dist3Squared, float dist4Squared,
                                                          Vector3f d1, Vector3f d2,
                                                          Vector3f dir1, Vector3f dir2,
                                                          Capsule capsule1, Capsule capsule2,
                                                          float len1Squared, boolean isCollide) {
        float minDistSquared = Math.min(Math.min(Math.min(dist1Squared, dist2Squared), dist3Squared), dist4Squared);
        float penetrationDepth = (float) (Math.sqrt(minDistSquared) - capsule1.radius - capsule2.radius);

        // Calculate collision points
        Vector3f minDistPoint = (minDistSquared == dist1Squared) ? capsule1.start :
                (minDistSquared == dist2Squared) ? capsule1.end :
                        (minDistSquared == dist3Squared) ? capsule2.start : capsule2.end;

        Vector3f collisionPointB = minDistPoint.add(d1.mul(dir1.dot(minDistPoint.sub(capsule1.start)) / len1Squared));
        Vector3f collisionPointA = minDistPoint.add(d2.mul(dir2.dot(minDistPoint.sub(capsule2.start)) / len1Squared));

        return new CollisionResult(isCollide, penetrationDepth, collisionPointA, collisionPointB);
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
