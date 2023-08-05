package collisionDetection.primitive;

import org.joml.Vector3f;

public class CollisionUtils {
    public static CollisionResult isAABBCollidingWithSphere(AABB aabb, Sphere sphere){
        Vector3f closestPoint = findClosestPointOnAABB(aabb, sphere.getCenter());

        // Calculate the squared distance between the closest point on the AABB and the center of the Sphere
        float distanceSquared = closestPoint.distanceSquared(sphere.getCenter());

        // Check if the squared distance is less than the squared radius of the Sphere
        boolean isColliding = distanceSquared <= sphere.getRadius() * sphere.getRadius();

        // Calculate the actual distance between the closest point and the center of the Sphere
        float distance = (float) Math.sqrt(distanceSquared);

        // Calculate the closest point on the Sphere's surface to the AABB
        Vector3f closestPointOnSphere = sphere.getCenter().add(closestPoint.sub(sphere.getCenter()).mul(sphere.getRadius()));

        return new CollisionResult(isColliding, distance, closestPoint, closestPointOnSphere);
    }

    // Helper method to find the closest point on an AABB to a given point
    private static Vector3f findClosestPointOnAABB(AABB aabb, Vector3f point) {
        float x = Math.max(aabb.getMin().x, Math.min(point.x, aabb.getMax().x));
        float y = Math.max(aabb.getMin().y, Math.min(point.y, aabb.getMax().y));
        float z = Math.max(aabb.getMin().z, Math.min(point.z, aabb.getMax().z));

        return new Vector3f(x, y, z);
    }
}
