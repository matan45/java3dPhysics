package collisionDetection.primitive;

import org.joml.Vector3f;

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


    @Override
    public String toString() {
        return "Cylinder{" +
                "center=" + center +
                ", radius=" + radius +
                ", height=" + height +
                '}';
    }
}
