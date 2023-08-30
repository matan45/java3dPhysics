package collisionDetection.primitive;

import math.Vector3f;
public class Sphere {
    private Vector3f center; // Center of the sphere
    private float radius; // Radius of the sphere

    public Sphere(Vector3f center, float radius) {
        this.center = center;
        this.radius = radius;
    }

    public Vector3f getCenter() {
        return center;
    }

    public void setCenter(Vector3f center) {
        this.center = center;
    }

    public float getRadius() {
        return radius;
    }

    public void setRadius(float radius) {
        this.radius = radius;
    }

    @Override
    public String toString() {
        return "Sphere{" +
                "center=" + center +
                ", radius=" + radius +
                '}';
    }

    // Method to find the closest point pair between two spheres
    public static boolean isSphereColliding(Sphere sphere1, Sphere sphere2) {
        float distanceSquared = sphere1.getCenter().distanceSquared(sphere2.getCenter());
        float radiusSum = sphere1.getRadius() + sphere2.getRadius();

        return distanceSquared <= (radiusSum * radiusSum);
    }
}
