package collisionDetection.primitive;

import org.joml.Vector3f;

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
    public static CollisionResult isSphereColliding(Sphere sphere1, Sphere sphere2) {
        Vector3f center1 = sphere1.getCenter();
        Vector3f center2 = sphere2.getCenter();
        float radius1 = sphere1.getRadius();
        float radius2 = sphere2.getRadius();

        Vector3f vecBetweenCenters = center2.sub(center1); // Vector from center1 to center2
        float distance = vecBetweenCenters.length(); // Distance between the centers

        // Check if the spheres overlap or are tangent
        if (distance <= radius1 + radius2) {
            // Spheres overlap or are tangent, return the centers as the closest points
            return new CollisionResult(true, distance, center1, center2);
        }

        // Spheres do not overlap, calculate the closest points on their surfaces
        float halfDistance = distance * 0.5f;
        float scaleFactor1 = (halfDistance + radius1) / distance; // Scale factor for vector from center1 to closest point on surface1
        float scaleFactor2 = (halfDistance + radius2) / distance; // Scale factor for vector from center2 to closest point on surface2

        Vector3f closestPoint1 = center1.add(vecBetweenCenters.mul(scaleFactor1));
        Vector3f closestPoint2 = center2.sub(vecBetweenCenters.mul(scaleFactor2));

        return new CollisionResult(false, distance - (radius1 + radius2), closestPoint1, closestPoint2);
    }
}
