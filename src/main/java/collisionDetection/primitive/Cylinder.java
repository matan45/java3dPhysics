package collisionDetection.primitive;

import math.Vector3f;

public class Cylinder {
    private Vector3f center;
    private float radius;
    private float height;

    public Cylinder(Vector3f center, float radius, float height) {
        this.center = center;
        this.radius = radius;
        this.height = height;
    }

    public Vector3f getCenter() {
        return center;
    }

    public float getRadius() {
        return radius;
    }

    public float getHeight() {
        return height;
    }

    public void setCenter(Vector3f center) {
        this.center = center;
    }

    public void setRadius(float radius) {
        this.radius = radius;
    }

    public void setHeight(float height) {
        this.height = height;
    }

    public static boolean isCylinderColliding(Cylinder cylinder1, Cylinder cylinder2) {
        Vector3f center1 = cylinder1.getCenter();
        Vector3f center2 = cylinder2.getCenter();

        float distanceSq = center1.distanceSquared(center2);
        float sumOfRadii = cylinder1.getRadius() + cylinder2.getRadius();
        float sumOfHeights = cylinder1.getHeight() + cylinder2.getHeight();

        // Check for collision by comparing the squared distances
        return distanceSq <= (sumOfRadii * sumOfRadii) && distanceSq <= (sumOfHeights * sumOfHeights);
    }

    public boolean isSegmentCollidingWithCylinderBody(Vector3f start, Vector3f end) {
        // Check if either end of the segment is within the cylinder's body
        boolean startInside = isPointInsideCylinder(start);
        boolean endInside = isPointInsideCylinder(end);

        // If the segment crosses the cylinder's central axis, there's a collision
        if (startInside || endInside) {
            return true;
        }

        // Check if the segment intersects the cylinder's body by checking the distance
        // of the segment's closest point to the cylinder's central axis
        Vector3f segmentDirection = end.sub(start);
        float t = segmentDirection.dot(getCenter().sub(start)) / segmentDirection.lengthSquared();
        t = Math.max(0, Math.min(1, t));
        Vector3f closestPointOnSegment = start.add(segmentDirection.mul(t));

        return closestPointOnSegment.sub(getCenter()).lengthSquared() <= getRadius() * getRadius();
    }

    private boolean isPointInsideCylinder(Vector3f point) {
        // Calculate the distance between the point and the cylinder's center in the XZ plane
        float distanceXZ = (float) Math.sqrt((point.x - getCenter().x) * (point.x - getCenter().x) +
                (point.z - getCenter().z) * (point.z - getCenter().z));

        // Check if the point is within the cylinder's radius and its Y coordinate is within the cylinder's height
        return distanceXZ <= getRadius() && Math.abs(point.y - getCenter().y) <= getHeight() / 2.0f;
    }


    @Override
    public String toString() {
        return "Cylinder{" +
                "center=" + center +
                ", radius=" + radius +
                ", height=" + height +
                '}';
    }
}
