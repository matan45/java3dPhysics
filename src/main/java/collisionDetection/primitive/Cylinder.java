package collisionDetection.primitive;

import collisionDetection.narrowPhase.Shape;
import math.Vector3f;

public class Cylinder implements Shape {
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

    @Override
    public boolean isPointInside(Vector3f point) {
        // Calculate the distance between the point and the cylinder's center in the x-z plane
        float distanceXZ = (float) Math.sqrt(Math.pow(point.x - center.x, 2) + Math.pow(point.z - center.z, 2));

        // Check if the point is within the radius and height bounds of the cylinder
        boolean isWithinRadius = distanceXZ <= radius;
        boolean isWithinHeight = point.y >= center.y && point.y <= center.y + height;

        // Return true if the point is within both radius and height bounds, false otherwise
        return isWithinRadius && isWithinHeight;
    }

    @Override
    public Vector3f closestPoint(Vector3f point) {
        // Calculate the direction from the cylinder's center to the given point
        Vector3f direction = new Vector3f(point.x - center.x, 0, point.z - center.z);

        // Clamp the direction's length to be within the cylinder's radius
        // Scale the normalized direction vector by the radius
        direction = direction.normalize().mul(radius);  // Normalize the direction vector

        // Calculate the closest point on the cylinder's axis
        Vector3f closestPointOnAxis = new Vector3f(center.x + direction.x, center.y, center.z + direction.z);

        // Calculate the closest point on the cylinder's surface
        float clampedY = Math.max(center.y, Math.min(center.y + height, point.y)); // Clamp y-coordinate

        return new Vector3f(closestPointOnAxis.x, clampedY, closestPointOnAxis.z);
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
        boolean startInside = isPointInside(start);
        boolean endInside = isPointInside(end);

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


    @Override
    public String toString() {
        return "Cylinder{" +
                "center=" + center +
                ", radius=" + radius +
                ", height=" + height +
                '}';
    }


}
