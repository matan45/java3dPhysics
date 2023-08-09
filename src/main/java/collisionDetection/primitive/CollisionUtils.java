package collisionDetection.primitive;

import org.joml.Vector3f;

public class CollisionUtils {
    public static boolean isAABBCollidingWithSphere(AABB aabb, Sphere sphere) {
        // Calculate the closest point to the sphere's center within the AABB
        float closestX = Math.max(aabb.getMin().x, Math.min(sphere.getCenter().x, aabb.getMax().x));
        float closestY = Math.max(aabb.getMin().y, Math.min(sphere.getCenter().y, aabb.getMax().y));
        float closestZ = Math.max(aabb.getMin().z, Math.min(sphere.getCenter().z, aabb.getMax().z));

        // Calculate the squared distance between the closest point and the sphere's center
        float distanceSquared = (closestX - sphere.getCenter().x) * (closestX - sphere.getCenter().x) +
                (closestY - sphere.getCenter().y) * (closestY - sphere.getCenter().y) +
                (closestZ - sphere.getCenter().z) * (closestZ - sphere.getCenter().z);

        return distanceSquared <= (sphere.getRadius() * sphere.getRadius());
    }
}
