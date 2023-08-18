package collisionDetection.primitive;

import org.joml.Vector3f;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class RayTest {
    @Test
    public void testRayIntersectsSphere() {
        Ray ray = new Ray(new Vector3f(0, 0, 0), new Vector3f(1, 0, 0)); // Example ray
        Sphere sphere = new Sphere(new Vector3f(2, 0, 0), 1.0f); // Example sphere

        assertTrue(Ray.isSphereCollide(ray, sphere));
    }

    @Test
    public void testRayMissesSphere() {
        Ray ray = new Ray(new Vector3f(0, 0, 0), new Vector3f(0, 1, 0)); // Example ray
        Sphere sphere = new Sphere(new Vector3f(3, 0, 0), 1.0f); // Example sphere

        assertFalse(Ray.isSphereCollide(ray, sphere));
    }


    @Test
    public void testRayIntersectsABB() {
        AABB aabb = new AABB(new Vector3f(0, 0, 0), new Vector3f(2, 2, 2));
        Ray ray = new Ray(new Vector3f(1, 1, 5), new Vector3f(0, 0, -1));

        // Test for collision
        assertTrue(Ray.isAABBCollide(ray, aabb));
    }

    @Test
    public void testRayMissesAABB() {
        AABB aabb = new AABB(new Vector3f(0, 0, 0), new Vector3f(2, 2, 2));
        Ray ray = new Ray(new Vector3f(1, 1, 5), new Vector3f(0, -1, 0));

        // Test for no collision
        assertFalse(Ray.isAABBCollide(ray, aabb));
    }

    @Test
    public void testRayIntersectsCapsule() {
        Capsule capsule = new Capsule(new Vector3f(0, 0, 0), new Vector3f(0, 4, 0), 1.0f);
        Ray ray = new Ray(new Vector3f(0, 5, 0), new Vector3f(0, -1, 0));

        // Test for collision
        assertTrue(Ray.isCapsuleCollide(ray, capsule));
    }

    @Test
    public void testRayMissesCapsule() {
        Capsule capsule = new Capsule(new Vector3f(0, 0, 0), new Vector3f(0, 4, 0), 1.0f);
        Ray ray = new Ray(new Vector3f(2, 2, 2), new Vector3f(0, 0, -1));

        // Test for no collision
        assertFalse(Ray.isCapsuleCollide(ray, capsule));
    }


    @Test
    public void testRayIntersectsCylinder() {
        Vector3f rayOrigin = new Vector3f(0, 0, 0);
        Vector3f rayDirection = new Vector3f(1, 0, 0);
        Ray ray = new Ray(rayOrigin, rayDirection);

        Vector3f cylinderCenter = new Vector3f(1, 0, 0);
        float cylinderRadius = 0.5f;
        float cylinderHeight = 2.0f;
        Cylinder cylinder = new Cylinder(cylinderCenter, cylinderRadius, cylinderHeight);

        boolean collision = Ray.isCylinderCollide(ray, cylinder);
        assertTrue(collision);
    }

    @Test
    public void testRayMissesCylinder() {
        Vector3f rayOrigin = new Vector3f(0, 0, 0);
        Vector3f rayDirection = new Vector3f(1, 0, 0);
        Ray ray = new Ray(rayOrigin, rayDirection);

        Vector3f cylinderCenter = new Vector3f(3, 0, 0); // Move cylinder out of the ray's path
        float cylinderRadius = 0.5f;
        float cylinderHeight = 2.0f;
        Cylinder cylinder = new Cylinder(cylinderCenter, cylinderRadius, cylinderHeight);

        boolean collision = Ray.isCylinderCollide(ray, cylinder);
        assertFalse(collision);
    }


    @Test
    public void testRayIntersectsPlane() {
        Vector3f rayOrigin = new Vector3f(0, 2, 0);
        Vector3f rayDirection = new Vector3f(0, -1, 0);
        Ray ray = new Ray(rayOrigin, rayDirection);

        Vector3f planeNormal = new Vector3f(0, 1, 0);
        float planeDistance = 1;
        Plane plane = new Plane(planeNormal, planeDistance);

        boolean collision = Ray.isPlaneCollide(ray, plane);
        assertTrue(collision);
    }

    @Test
    public void testRayMissesPlane() {
        Vector3f rayOrigin = new Vector3f(0, 0, 0);
        Vector3f rayDirection = new Vector3f(1, 0, 0);
        Ray ray = new Ray(rayOrigin, rayDirection);

        Vector3f planeNormal = new Vector3f(0, 1, 0);
        float planeDistance = 2;
        Plane plane = new Plane(planeNormal, planeDistance);

        boolean collision = Ray.isPlaneCollide(ray, plane);
        assertFalse(collision);
    }

    @Test
    public void testRayIntersectsOBB() {
        OBB obb = new OBB(new Vector3f(0, 0, 0), new Vector3f(1, 1, 1));
        Ray ray = new Ray(new Vector3f(0, -1, 0), new Vector3f(0, 1, 0));

        // Test for collision
        assertTrue(Ray.isOBBCollide(ray, obb));
    }

    @Test
    public void testRayMissesOBB() {
        OBB obb = new OBB(new Vector3f(0, 0, 0), new Vector3f(2, 2, 2));
        Ray ray = new Ray(new Vector3f(0, 3, 0), new Vector3f(0, 1, 0));

        // Test for no collision
        assertFalse(Ray.isOBBCollide(ray, obb));
    }

}