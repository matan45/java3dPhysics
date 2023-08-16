package collisionDetection.primitive;

import org.joml.Vector3f;

public class Ray {

    private Vector3f origin;
    private Vector3f direction;

    public Ray(Vector3f origin, Vector3f direction) {
        this.origin = origin;
        this.direction = direction.normalize();
    }

    public Vector3f getOrigin() {
        return origin;
    }

    public void setOrigin(Vector3f origin) {
        this.origin = origin;
    }

    public Vector3f getDirection() {
        return direction;
    }

    public void setDirection(Vector3f direction) {
        this.direction = direction;
    }

    public Vector3f closestPoint(Vector3f point) {
        float t = point.sub(origin).dot(direction);
        t = Math.max(t, 0.0f);
        return origin.add(direction.mul(t));
    }

    public static boolean isSphereCollide(Ray ray, Sphere sphere) {
        Vector3f oc = ray.getOrigin().sub(sphere.getCenter());
        float a = ray.getDirection().dot(ray.getDirection());
        float b = 2.0f * oc.dot(ray.getDirection());
        float c = oc.dot(oc) - sphere.getRadius() * sphere.getRadius();
        float discriminant = b * b - 4 * a * c;

        // Intersection
        return !(discriminant < 0); // No intersection
    }

    public static boolean isAABBCollide(Ray ray, AABB aabb) {
        // Find the two intersection points on the AABB using the ray's closest point method
        Vector3f p1 = ray.closestPoint(aabb.getMin());
        Vector3f p2 = ray.closestPoint(aabb.getMax());

        // Check if the intersection points are within the AABB's boundaries
        boolean betweenX = (p1.x >= aabb.getMin().x && p1.x <= aabb.getMax().x) || (p2.x >= aabb.getMin().x && p2.x <= aabb.getMax().x);
        boolean betweenY = (p1.y >= aabb.getMin().y && p1.y <= aabb.getMax().y) || (p2.y >= aabb.getMin().y && p2.y <= aabb.getMax().y);
        boolean betweenZ = (p1.z >= aabb.getMin().z && p1.z <= aabb.getMax().z) || (p2.z >= aabb.getMin().z && p2.z <= aabb.getMax().z);

        // If there's an intersection along all axes, the AABB and ray collide
        return betweenX && betweenY && betweenZ;
    }

    public static boolean isCapsuleCollide(Ray ray, Capsule capsule) {
        Vector3f rayOrigin = ray.getOrigin();
        Vector3f rayDirection = ray.getDirection();

        Vector3f capsuleStart = capsule.getStart();
        Vector3f capsuleEnd = capsule.getEnd();
        float capsuleRadius = capsule.getRadius();

        Vector3f ab = capsuleEnd.sub(capsuleStart);
        Vector3f ac = rayOrigin.sub(capsuleStart);

        float t = ac.dot(rayDirection);
        if (t < 0) t = 0;
        if (t > 1) t = 1;

        Vector3f h = rayDirection.mul(t).sub(ac);

        float a = ab.dot(ab);
        float b = 2 * h.dot(ab);
        float c = h.dot(h) - capsuleRadius * capsuleRadius;

        float discriminant = b * b - 4 * a * c;

        if (discriminant < 0) {
            return false; // No collision
        } else {
            // Calculate the two potential intersection points
            float sqrtDiscriminant = (float) Math.sqrt(discriminant);
            float t1 = (-b - sqrtDiscriminant) / (2 * a);
            float t2 = (-b + sqrtDiscriminant) / (2 * a);

            return (t1 >= 0 && t1 <= 1) || (t2 >= 0 && t2 <= 1); // Collision detected
        }// No collision
    }

    public static boolean isCylinderCollide(Ray ray, Cylinder cylinder) {
        return false;
    }

    public static boolean isOBBCollide(Ray ray, OBB obb) {
        return false;
    }

    public static boolean isPlaneCollide(Ray ray, Plane plane) {
        return false;
    }

    public static boolean isTriangleCollide(Ray ray, Triangle triangle) {
        return false;
    }


    @Override
    public String toString() {
        return "Ray{" +
                "origin=" + origin +
                ", direction=" + direction +
                '}';
    }
}
