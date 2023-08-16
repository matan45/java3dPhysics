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
        // Calculate the direction from the start to end of the capsule
        Vector3f capsuleDirection = capsule.getEnd().sub(capsule.getStart());

        // Calculate the vector from the ray's origin to the start of the capsule
        Vector3f startToOrigin = ray.getOrigin().sub(capsule.getStart());

        // Project the vector from start to origin onto the capsule direction
        float t = startToOrigin.dot(capsuleDirection) / capsuleDirection.lengthSquared();

        // Clamp the parameter 't' to the range [0, 1]
        t = Math.max(0.0f, Math.min(1.0f, t));

        // Calculate the closest point on the capsule
        Vector3f closestPointOnCapsule = capsule.getStart().add(capsuleDirection.mul(t));

        // Calculate the closest point on the ray to the capsule
        Vector3f closestPointOnRay = ray.closestPoint(closestPointOnCapsule);

        // Calculate the squared distance between the two closest points
        float squaredDistance = closestPointOnCapsule.sub(closestPointOnRay).lengthSquared();

        // If the squared distance is less than or equal to the capsule's radius squared, there's a collision
        float radiusSquared = capsule.getRadius() * capsule.getRadius();
        return squaredDistance <= radiusSquared;
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
