package collisionDetection.primitive;

import math.Vector3f;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class CollisionDetectionTest {


    @Test
    void testAABBCollidingWithSphere() {
        Vector3f min = new Vector3f(-1, -1, -1);
        Vector3f max = new Vector3f(1, 1, 1);
        AABB aabb = new AABB(min, max);

        Sphere sphere = new Sphere(new Vector3f(0, 1.5f, 0), 1.0f);

        boolean result = CollisionDetection.isSphereCollidingWithAABB(sphere, aabb);

        assertTrue(result, "AABB Sphere should be colliding");
    }

    @Test
    void testAABBNotCollidingWithSphere() {
        Vector3f min = new Vector3f(-1, -1, -1);
        Vector3f max = new Vector3f(1, 1, 1);
        AABB aabb = new AABB(min, max);

        Sphere sphere = new Sphere(new Vector3f(2, 2, 2), 0.5f);
        boolean result = CollisionDetection.isSphereCollidingWithAABB(sphere, aabb);

        assertFalse(result, "AABB Sphere should not be colliding");
    }


    @Test
    public void testSphereCollidingWithPlane() {
        Sphere sphere = new Sphere(new Vector3f(0.0f, 4.0f, 0.0f), 2.0f);
        Plane plane = new Plane(new Vector3f(0.0f, 1.0f, 0.0f), 3.0f);

        boolean collision = CollisionDetection.isSphereCollidingWithPlane(sphere, plane);

        assertTrue(collision);
    }

    @Test
    public void testSphereNotCollidingWithPlane() {
        Sphere sphere = new Sphere(new Vector3f(0.0f, 5.0f, 0.0f), 2.0f);
        Plane plane = new Plane(new Vector3f(0.0f, 0.0f, 0.0f), 10.0f);

        boolean collision = CollisionDetection.isSphereCollidingWithPlane(sphere, plane);

        assertFalse(collision);
    }

    @Test
    public void testSphereCollisionWithTriangle() {
        // Create a sphere and a triangle for testing
        Sphere sphere = new Sphere(new Vector3f(0, 0, 0), 1.0f);
        Triangle triangle = new Triangle(
                new Vector3f(-1, -1, 0),
                new Vector3f(1, -1, 0),
                new Vector3f(0, 1, 0)
        );

        assertTrue(CollisionDetection.isSphereCollidingWithTriangle(sphere, triangle), "Sphere should be colliding with the triangle");
    }

    @Test
    public void testSphereNotCollisionWithTriangle() {
        // Create a sphere and a triangle for testing
        Sphere sphere = new Sphere(new Vector3f(0, 0, 0), 0.5f);
        Triangle triangle = new Triangle(
                new Vector3f(1, 0, 0),
                new Vector3f(2, 0, 1),
                new Vector3f(1, 1, 0)
        );

        assertFalse(CollisionDetection.isSphereCollidingWithTriangle(sphere, triangle), "Sphere should not be colliding with the triangle");
    }

    @Test
    void testOBBCollidingWithSphere() {
        Vector3f center1 = new Vector3f(0.0f, 0.0f, 0.0f);
        Vector3f halfExtents1 = new Vector3f(1.0f, 1.0f, 1.0f);
        OBB obb = new OBB(center1, halfExtents1);


        Sphere sphere = new Sphere(new Vector3f(0, 1.5f, 0), 1.0f);

        boolean result = CollisionDetection.isSphereCollidingWithOBB(sphere, obb);

        assertTrue(result, "OBB Sphere should be colliding");
    }

    @Test
    void testOBBNotCollidingWithSphere() {
        Vector3f center1 = new Vector3f(8.0f, 8.0f, 8.0f);
        Vector3f halfExtents1 = new Vector3f(1.0f, 1.0f, 1.0f);
        OBB obb = new OBB(center1, halfExtents1);


        Sphere sphere = new Sphere(new Vector3f(2, 2, 2), 0.5f);
        boolean result = CollisionDetection.isSphereCollidingWithOBB(sphere, obb);

        assertFalse(result, "OBB Sphere should not be colliding");
    }

    @Test
    public void testOBBCollidingWithAABB() {
        Vector3f min = new Vector3f(-1.0f, -1.0f, -1.0f);
        Vector3f max = new Vector3f(1.0f, 1.0f, 1.0f);
        AABB aabb = new AABB(min, max);

        Vector3f center = new Vector3f(0.0f, 0.0f, 0.0f);
        Vector3f halfExtents = new Vector3f(1.0f, 1.0f, 1.0f);
        OBB obb = new OBB(center, halfExtents);

        boolean collision = CollisionDetection.isAABBCollidingWithOBB(aabb, obb);

        assertTrue(collision,"OBB AABB should be colliding");
    }

    @Test
    public void testOBBNotCollidingWithAABB() {
        Vector3f min = new Vector3f(-1.0f, -1.0f, -1.0f);
        Vector3f max = new Vector3f(1.0f, 1.0f, 1.0f);
        AABB aabb = new AABB(min, max);

        Vector3f center = new Vector3f(3.0f, 3.0f, 3.0f);
        Vector3f halfExtents = new Vector3f(1.0f, 1.0f, 1.0f);
        OBB obb = new OBB(center, halfExtents);

        boolean collision = CollisionDetection.isAABBCollidingWithOBB(aabb, obb);

        assertFalse(collision,"OBB AABB should not be colliding");
    }

    @Test
    public void testSphereCollidingWithCapsule() {
        Vector3f sphereCenter = new Vector3f(0.0f, 0.0f, 0.0f);
        float sphereRadius = 1.0f;
        Sphere sphere = new Sphere(sphereCenter, sphereRadius);

        Vector3f capsuleStart = new Vector3f(0.0f, 0.0f, 0.0f);
        Vector3f capsuleEnd = new Vector3f(0.0f, 2.0f, 0.0f);
        float capsuleRadius = 1.0f;
        Capsule capsule = new Capsule(capsuleStart, capsuleEnd, capsuleRadius);

        boolean collision = CollisionDetection.isSphereCollidingWithCapsule(sphere, capsule);

        assertTrue(collision,"Sphere Capsule should be colliding");
    }

    @Test
    public void testSphereNotCollidingWithCapsule() {
        Vector3f sphereCenter = new Vector3f(0.0f, 0.0f, 0.0f);
        float sphereRadius = 1.0f;
        Sphere sphere = new Sphere(sphereCenter, sphereRadius);

        Vector3f capsuleStart = new Vector3f(0.0f, 2.0f, 0.0f);
        Vector3f capsuleEnd = new Vector3f(0.0f, 4.0f, 0.0f);
        float capsuleRadius = 1.0f;
        Capsule capsule = new Capsule(capsuleStart, capsuleEnd, capsuleRadius);

        boolean collision = CollisionDetection.isSphereCollidingWithCapsule(sphere, capsule);

        assertFalse(collision,"Sphere Capsule should not be colliding");
    }

    @Test
    public void testSphereCollidingWithCylinder() {
        Vector3f sphereCenter = new Vector3f(0.0f, 0.0f, 0.0f);
        float sphereRadius = 1.0f;
        Sphere sphere = new Sphere(sphereCenter, sphereRadius);

        Vector3f cylinderCenter = new Vector3f(0.0f, 0.0f, 0.0f);
        float cylinderRadius = 1.0f;
        float cylinderHeight = 2.0f;
        Cylinder cylinder = new Cylinder(cylinderCenter, cylinderRadius, cylinderHeight);

        boolean collision = CollisionDetection.isSphereCollidingWithCylinder(sphere, cylinder);

        assertTrue(collision,"Sphere Cylinder should be colliding");
    }

    @Test
    public void testSphereNotCollidingWithCylinder() {
        Vector3f sphereCenter = new Vector3f(0.0f, 0.0f, 0.0f);
        float sphereRadius = 1.0f;
        Sphere sphere = new Sphere(sphereCenter, sphereRadius);

        Vector3f cylinderCenter = new Vector3f(0.0f, 5.0f, 0.0f);
        float cylinderRadius = 1.0f;
        float cylinderHeight = 1.0f;
        Cylinder cylinder = new Cylinder(cylinderCenter, cylinderRadius, cylinderHeight);

        boolean collision = CollisionDetection.isSphereCollidingWithCylinder(sphere, cylinder);

        assertFalse(collision,"Sphere Cylinder should not be colliding");
    }
}
