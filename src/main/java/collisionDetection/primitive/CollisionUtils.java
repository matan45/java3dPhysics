package collisionDetection.primitive;

import org.joml.Vector3f;

public class CollisionUtils {
    public static boolean isAABBCollidingWithSphere(Sphere sphere, AABB aabb) {
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

    public static boolean isSphereCollidingWithTriangle(Sphere sphere, Triangle triangle) {
        Vector3f closestPoint = triangle.closestPoint(sphere.getCenter());
        Vector3f subSq = sphere.getCenter().sub(closestPoint);
        float distSq = subSq.dot(subSq);
        float radiusSq = sphere.getRadius() * sphere.getRadius();
        return distSq < radiusSq;
    }

    public static boolean isSphereCollidingWithOBB(Sphere sphere, OBB obb) {
        Vector3f closestPoint = obb.closestPoint(sphere.getCenter());
        Vector3f subSq = sphere.getCenter().sub(closestPoint);
        float distSq = subSq.dot(subSq);
        float radiusSq = sphere.getRadius() * sphere.getRadius();
        return distSq < radiusSq;
    }
}
