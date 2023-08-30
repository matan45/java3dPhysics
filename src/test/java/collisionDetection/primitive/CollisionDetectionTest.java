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

        boolean result = CollisionDetection.isSphereCollidingWithPlane(sphere, plane);

        assertTrue(result,"Sphere Plane should be colliding");
    }

    @Test
    public void testSphereNotCollidingWithPlane() {
        Sphere sphere = new Sphere(new Vector3f(0.0f, 5.0f, 0.0f), 2.0f);
        Plane plane = new Plane(new Vector3f(1.0f, 0.0f, 0.0f), 10.0f);

        boolean result = CollisionDetection.isSphereCollidingWithPlane(sphere, plane);

        assertFalse(result,"Sphere Plane should not be colliding");
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
        boolean result = CollisionDetection.isSphereCollidingWithTriangle(sphere, triangle);
        assertTrue(result, "Sphere should be colliding with the triangle");
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

        boolean result = CollisionDetection.isSphereCollidingWithTriangle(sphere, triangle);

        assertFalse(result, "Sphere should not be colliding with the triangle");
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

    @Test
    void testPlaneCollidingWithAABB() {
        Plane plane = new Plane(new Vector3f(0, 1, 0), 0); // Plane with normal (0, 1, 0) and distance 0
        AABB aabb = new AABB(new Vector3f(-1, -1, -1), new Vector3f(1, 1, 1)); // AABB centered at (0, 0, 0)

        boolean result = CollisionDetection.isPlaneCollidingWithAABB(plane, aabb);
        assertTrue(result, "AABB and plane should be colliding");
    }

    @Test
    void testPlaneNotCollidingWithAABB() {
        Plane plane = new Plane(new Vector3f(0, 1, 0), 0); // Plane with normal (0, 1, 0) and distance 0
        AABB aabb = new AABB(new Vector3f(-2, -2, -2), new Vector3f(-1, -1, -1)); // AABB above the plane

        boolean result = CollisionDetection.isPlaneCollidingWithAABB(plane, aabb);
        assertFalse(result, "AABB and plane should not be colliding");
    }

    @Test
    void testCapsuleCollidingWithAABB() {
        Capsule capsule = new Capsule(new Vector3f(0, 0, 0), new Vector3f(0, 2, 0), 1.0f);
        AABB aabb = new AABB(new Vector3f(-1, -1, -1), new Vector3f(1, 1, 1));

        boolean result = CollisionDetection.isCapsuleCollidingWithAABB(capsule, aabb);
        assertTrue(result, "Capsule and AABB should be colliding");
    }

    @Test
    void testCapsuleNotCollidingWithAABB() {
        Capsule capsule = new Capsule(new Vector3f(1, 1, 3), new Vector3f(1, 2, 3), 1.0f);
        AABB aabb = new AABB(new Vector3f(-1, -1, -1), new Vector3f(1, 1, 1));

        boolean result = CollisionDetection.isCapsuleCollidingWithAABB(capsule, aabb);
        assertFalse(result, "Capsule and AABB should not be colliding");
    }

    @Test
    void testCylinderCollidingWithAABB() {
        Cylinder cylinder = new Cylinder(new Vector3f(0, 1, 0), 1.0f, 4.0f);
        AABB aabb = new AABB(new Vector3f(-1, 0, -1), new Vector3f(1, 3, 1));

        boolean result = CollisionDetection.isCylinderCollidingWithAABB(cylinder, aabb);
        assertTrue(result, "Cylinder and AABB should be colliding");
    }

    @Test
    void testCylinderNotCollidingWithAABB() {
        Cylinder cylinder = new Cylinder(new Vector3f(0, 5, 0), 1.0f, 2.0f);
        AABB aabb = new AABB(new Vector3f(-1, 0, -1), new Vector3f(1, 3, 1));

        boolean result = CollisionDetection.isCylinderCollidingWithAABB(cylinder, aabb);
        assertFalse(result, "Cylinder and AABB should not be colliding");
    }

    @Test
    void testCapsuleCollidingWithCylinder() {
        Capsule capsule = new Capsule(new Vector3f(0, 1, 0), new Vector3f(0, 3, 0), 0.5f);
        Cylinder cylinder = new Cylinder(new Vector3f(0, 2, 0), 1.0f, 4.0f);

        boolean result = CollisionDetection.isCapsuleCollidingWithCylinder(capsule, cylinder);
        assertTrue(result, "Capsule and Cylinder should be colliding");
    }

    @Test
    void testCapsuleNotCollidingWithCylinder() {
        Capsule capsule = new Capsule(new Vector3f(0, 10, 0), new Vector3f(0, 12, 0), 1f);
        Cylinder cylinder = new Cylinder(new Vector3f(0, 2, 0), 1.0f, 4.0f);

        boolean result = CollisionDetection.isCapsuleCollidingWithCylinder(capsule, cylinder);
        assertFalse(result, "Capsule and Cylinder should not be colliding");
    }

    @Test
    void testCylinderCollidingWithPlane() {
        Cylinder cylinder = new Cylinder(new Vector3f(0, 1, 0), 1.0f, 2.0f);
        Plane plane = new Plane(new Vector3f(0, 1, 0), 0); // Plane with normal (0, 1, 0) and distance 0

        boolean result = CollisionDetection.isCylinderCollidingWithPlane(cylinder, plane);
        assertTrue(result, "Cylinder and Plane should be colliding");
    }

    @Test
    void testCylinderNotCollidingWithPlane() {
        Cylinder cylinder = new Cylinder(new Vector3f(0, 5, 0), 1.0f, 2.0f);
        Plane plane = new Plane(new Vector3f(0, 1, 0), 0); // Plane with normal (0, 1, 0) and distance 0

        boolean result = CollisionDetection.isCylinderCollidingWithPlane(cylinder, plane);
        assertFalse(result, "Cylinder and Plane should not be colliding");
    }

    @Test
    void testCapsuleCollidingWithPlane() {
        Capsule capsule = new Capsule(new Vector3f(0, 1, 0), new Vector3f(0, 3, 0), 0.5f);
        Plane plane = new Plane(new Vector3f(0, 1, 0), 0); // Plane with normal (0, 1, 0) and distance 0

        boolean result = CollisionDetection.isCapsuleCollidingWithPlane(capsule, plane);
        assertTrue(result, "Capsule and Plane should be colliding");
    }

    @Test
    void testCapsuleNotCollidingWithPlane() {
        Capsule capsule = new Capsule(new Vector3f(0, 5, 0), new Vector3f(0, 7, 0), 0.5f);
        Plane plane = new Plane(new Vector3f(0, 1, 0), 0); // Plane with normal (0, 1, 0) and distance 0

        boolean result = CollisionDetection.isCapsuleCollidingWithPlane(capsule, plane);
        assertFalse(result, "Capsule and Plane should not be colliding");
    }

    @Test
    void testCylinderCollidingWithTriangle() {
        Cylinder cylinder = new Cylinder(new Vector3f(0, 0, 0), 1.0f, 2.0f);

        Triangle triangle = new Triangle(new Vector3f(0, 0, 0), new Vector3f(1, 0, 0), new Vector3f(0, 1, 0));

        boolean collision = CollisionDetection.isCylinderCollidingWithTriangle(cylinder, triangle);
        assertTrue(collision, "Expected collision between cylinder and triangle");
    }

    @Test
    void testCylinderNotCollidingWithTriangle() {
        Cylinder cylinder = new Cylinder(new Vector3f(5, 5, 5), 1.0f, 2.0f);

        Triangle triangle = new Triangle(new Vector3f(-3, 0, 0), new Vector3f(-2, 0, 0), new Vector3f(-2, -1, 0));

        boolean collision = CollisionDetection.isCylinderCollidingWithTriangle(cylinder, triangle);
        assertFalse(collision, "Expected no collision between cylinder and triangle");
    }

    @Test
    void testCylinderCollidingWithOBB() {
        Cylinder cylinder = new Cylinder(new Vector3f(0, 0, 0), 1.0f, 2.0f);
        OBB obb = new OBB(new Vector3f(1, 0, 0), new Vector3f(1, 1, 1));

        boolean result = CollisionDetection.isCylinderCollidingWithOBB(cylinder, obb);
        assertTrue(result, "Collision should be detected");
    }

    @Test
    void testCylinderNotCollidingWithOBB() {
        Cylinder cylinder = new Cylinder(new Vector3f(0, 0, 0), 1.0f, 2.0f);
        OBB obb = new OBB(new Vector3f(4, 4, 4), new Vector3f(1, 1, 1));

        boolean result = CollisionDetection.isCylinderCollidingWithOBB(cylinder, obb);
        assertFalse(result, "No collision should be detected");
    }


    @Test
    public void testAABBCollidingWithTriangle() {
        Vector3f aabbMin = new Vector3f(0, 0, 0);
        Vector3f aabbMax = new Vector3f(2, 2, 2);
        AABB aabb = new AABB(aabbMin, aabbMax);

        Vector3f vertex1 = new Vector3f(1, 1, 1);
        Vector3f vertex2 = new Vector3f(3, 1, 1);
        Vector3f vertex3 = new Vector3f(1, 3, 1);
        Triangle triangle = new Triangle(vertex1, vertex2, vertex3);

        assertTrue(CollisionDetection.isTriangleCollidingWithAABB(triangle, aabb));
    }

    @Test
    public void testAABBNotCollidingWithTriangle() {
        Vector3f aabbMin = new Vector3f(-2, -2, 0);
        Vector3f aabbMax = new Vector3f(0, 0, 0);
        AABB aabb = new AABB(aabbMin, aabbMax);

        Vector3f vertex1 = new Vector3f(3, 3, 0);
        Vector3f vertex2 = new Vector3f(4, 4, 0);
        Vector3f vertex3 = new Vector3f(5, 5, 0);
        Triangle triangle = new Triangle(vertex1, vertex2, vertex3);

        assertFalse(CollisionDetection.isTriangleCollidingWithAABB(triangle, aabb));
    }

    @Test
    void testPlaneCollidingWithTriangle() {
        Vector3f normal = new Vector3f(0, 1, 0);
        Plane plane = new Plane(normal, 0);

        Vector3f vertex1 = new Vector3f(0, 0, 0);
        Vector3f vertex2 = new Vector3f(1, 0, 0);
        Vector3f vertex3 = new Vector3f(0.5f, 1, 0);
        Triangle triangle = new Triangle(vertex1, vertex2, vertex3);

        assertTrue(CollisionDetection.isTriangleCollidingWithPlane(triangle, plane));
    }

    @Test
    void testPlaneNotCollidingWithTriangle() {
        Vector3f normal = new Vector3f(0, 1, 0);
        Plane plane = new Plane(normal, 0);

        Vector3f vertex1 = new Vector3f(0, 0, 0);
        Vector3f vertex2 = new Vector3f(1, 0, 0);
        Vector3f vertex3 = new Vector3f(2, 0, 0);
        Triangle triangle = new Triangle(vertex1, vertex2, vertex3);

        assertFalse(CollisionDetection.isTriangleCollidingWithPlane(triangle, plane));
    }


    @Test
    public void testCapsuleCollidingWithOBB() {
        // Create a colliding capsule and OBB
        Capsule capsule = new Capsule(new Vector3f(0, 0, 0), new Vector3f(0, 1, 0), 0.5f);
        OBB obb = new OBB(new Vector3f(0, 0.5f, 0), new Vector3f(0.5f, 0.5f, 0.5f));

        assertTrue(CollisionDetection.isCapsuleCollidingWithOBB(capsule, obb));
    }

    @Test
    public void testCapsuleNotCollidingWithOBB() {
        // Create a non-colliding capsule and OBB
        Capsule capsule = new Capsule(new Vector3f(3, 3, 0), new Vector3f(5, 5, 0), 0.5f);
        OBB obb = new OBB(new Vector3f(0, 0, 0), new Vector3f(0.5f, 0.5f, 0.5f));

        assertFalse(CollisionDetection.isCapsuleCollidingWithOBB(capsule, obb));
    }

    @Test
    void testCapsuleCollidingWithTriangle() {
        Vector3f capsuleStart = new Vector3f(0, 0, -1);
        Vector3f capsuleEnd = new Vector3f(0, 2, 1);
        float capsuleRadius = 0.5f;
        Capsule capsule = new Capsule(capsuleStart, capsuleEnd, capsuleRadius);

        Vector3f vertex1 = new Vector3f(-1, 0, 0);
        Vector3f vertex2 = new Vector3f(0, 1, 0);
        Vector3f vertex3 = new Vector3f(1, 1, 0);
        Triangle triangle = new Triangle(vertex1, vertex2, vertex3);

        assertTrue(CollisionDetection.isCapsuleCollidingWithTriangle(capsule, triangle));
    }

    @Test
    void testCapsuleNoCollidingWithTriangle() {
        Vector3f capsuleStart = new Vector3f(0, 0, 0);
        Vector3f capsuleEnd = new Vector3f(0, 3, 0);
        float capsuleRadius = 0.5f;
        Capsule capsule = new Capsule(capsuleStart, capsuleEnd, capsuleRadius);

        Vector3f vertex1 = new Vector3f(-1, -1, 0);
        Vector3f vertex2 = new Vector3f(-1, 2, 0);
        Vector3f vertex3 = new Vector3f(-3, -1, 0);
        Triangle triangle = new Triangle(vertex1, vertex2, vertex3);

        assertFalse(CollisionDetection.isCapsuleCollidingWithTriangle(capsule, triangle));
    }
}
