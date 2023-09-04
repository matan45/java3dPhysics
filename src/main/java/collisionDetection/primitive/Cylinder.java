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


    @Override
    public String toString() {
        return "Cylinder{" +
                "center=" + center +
                ", radius=" + radius +
                ", height=" + height +
                '}';
    }


}
