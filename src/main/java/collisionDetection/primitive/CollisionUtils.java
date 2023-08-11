package collisionDetection.primitive;

import org.joml.Vector3f;

public class CollisionUtils {
    public static boolean isAABBCollidingWithSphere(AABB aabb, Sphere sphere) {
        Vector3f closestPoint = aabb.closestPoint(sphere.getCenter());
        Vector3f subSq = sphere.getCenter().sub(closestPoint);
        float distSq = subSq.dot(subSq);
        float radiusSq = sphere.getRadius() * sphere.getRadius();
        return distSq < radiusSq;
    }

    public static boolean isSphereCollidingWithPlane(Sphere sphere, Plane plane) {
        Vector3f closestPoint = plane.closestPoint(sphere.getCenter());
        Vector3f subSq = sphere.getCenter().sub(closestPoint);
        float distSq = subSq.dot(subSq);
        float radiusSq = sphere.getRadius() * sphere.getRadius();
        return distSq < radiusSq;
    }
}
