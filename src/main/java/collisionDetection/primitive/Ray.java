package collisionDetection.primitive;

import org.joml.Vector3f;

import java.util.Arrays;

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

        if (discriminant < 0)
            return false; // No collision

        // Calculate the two potential intersection points
        float sqrtDiscriminant = (float) Math.sqrt(discriminant);
        float t1 = (-b - sqrtDiscriminant) / (2 * a);
        float t2 = (-b + sqrtDiscriminant) / (2 * a);

        return (t1 >= 0 && t1 <= 1) || (t2 >= 0 && t2 <= 1);
    }

    public static boolean isCylinderCollide(Ray ray, Cylinder cylinder) {
        Vector3f rayOrigin = ray.getOrigin();
        Vector3f rayDirection = ray.getDirection();

        Vector3f cylinderCenter = cylinder.getCenter();
        float cylinderRadius = cylinder.getRadius();
        float cylinderHeight = cylinder.getHeight();

        Vector3f oc = rayOrigin.sub(cylinderCenter);

        float a = rayDirection.lengthSquared();
        float b = 2 * oc.dot(rayDirection);
        float c = oc.lengthSquared() - cylinderRadius * cylinderRadius;

        float discriminant = b * b - 4 * a * c;

        if (discriminant >= 0) {
            float sqrtDiscriminant = (float) Math.sqrt(discriminant);
            float t1 = (-b - sqrtDiscriminant) / (2 * a);
            float t2 = (-b + sqrtDiscriminant) / (2 * a);

            // Check if either intersection point is within the bounds of the cylinder's height
            return (t1 >= 0 && t1 <= cylinderHeight) || (t2 >= 0 && t2 <= cylinderHeight); // Collision detected
        }

        return false; // No collision
    }

    public static boolean isOBBCollide(Ray ray, OBB obb) {
        Vector3f rayOrigin = ray.getOrigin();
        Vector3f rayDirection = ray.getDirection();

        Vector3f obbCenter = obb.getCenter();
        Vector3f[] obbAxis = obb.getAxis();
        Vector3f size = obb.getHalfExtents();

        // Vector from ray origin to OBB center
        Vector3f p = obbCenter.sub(rayOrigin);

        // Calculate dot products between OBB axes and ray direction
        Vector3f f = new Vector3f(
                obbAxis[0].dot(rayDirection),
                obbAxis[1].dot(rayDirection),
                obbAxis[2].dot(rayDirection));

        // Calculate dot products between OBB axes and vector p
        Vector3f e = new Vector3f(
                obbAxis[0].dot(p),
                obbAxis[1].dot(p),
                obbAxis[2].dot(p));


        float[] t = new float[]{
                0, 0, 0, 0, 0, 0
        };
        // Calculate intersection intervals for each OBB axis
        for (int i = 0; i < 3; i++) {
            t[i * 2] = (e.get(i) + size.get(i)) / f.get(i); // min
            t[i * 2 + 1] = (e.get(i) - size.get(i)) / f.get(i); // max
        }

        // Calculate min and max for all intervals
        float min = Math.max(
                Math.max(
                        Math.min(t[0], t[1]),
                        Math.min(t[2], t[3])),
                Math.min(t[4], t[5])
        );
        float max = Math.min(
                Math.min(
                        Math.max(t[0], t[1]),
                        Math.max(t[2], t[3])),
                Math.max(t[4], t[5])
        );

        return !(min > max) && !(max < 0);// Collision detected
    }

    public static boolean isPlaneCollide(Ray ray, Plane plane) {
        Vector3f rayOrigin = ray.getOrigin();
        Vector3f rayDirection = ray.getDirection();

        Vector3f planeNormal = plane.getNormal();
        float planeDistance = plane.getDistance();

        float nd = rayDirection.dot(planeNormal);
        float pn = rayOrigin.dot(planeNormal);

        if (nd >= 0.0f)
            return false;

        float t = (planeDistance - pn) / nd;

        return t >= 0; // Collision detected
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
