package collisionDetection.narrowPhase.gjk;

import collisionDetection.primitive.*;
import math.Vector3f;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;


class GJKTest {
    static GJK gjk = new GJK();

    @Test
    public void testOBBCollidingWithAABB() {
        Vector3f min = new Vector3f(-1.0f, -1.0f, -1.0f);
        Vector3f max = new Vector3f(1.0f, 1.0f, 1.0f);
        AABB aabb = new AABB(min, max);

        Vector3f center = new Vector3f(0.0f, 0.0f, 0.0f);
        Vector3f halfExtents = new Vector3f(1.0f, 1.0f, 1.0f);
        OBB obb = new OBB(center, halfExtents);

        boolean collision = gjk.isCollide(aabb, obb).isColliding();

        assertTrue(collision, "OBB AABB should be colliding");
    }

    @Test
    public void testOBBNotCollidingWithAABB() {
        Vector3f min = new Vector3f(-1.0f, -1.0f, -1.0f);
        Vector3f max = new Vector3f(1.0f, 1.0f, 1.0f);
        AABB aabb = new AABB(min, max);

        Vector3f center = new Vector3f(3.0f, 3.0f, 3.0f);
        Vector3f halfExtents = new Vector3f(1.0f, 1.0f, 1.0f);
        OBB obb = new OBB(center, halfExtents);

        boolean collision = gjk.isCollide(aabb, obb).isColliding();

        assertFalse(collision, "OBB AABB should not be colliding");
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

        assertTrue(gjk.isCollide(triangle, aabb).isColliding());
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

        assertFalse(gjk.isCollide(triangle, aabb).isColliding());
    }


    @Test
    void testNonCollidingPolyhedra() {
        ConvexPolyhedron polyhedron1 = createNonCollidingPolyhedron1();
        ConvexPolyhedron polyhedron2 = createNonCollidingPolyhedron2();

        assertFalse(gjk.isCollide(polyhedron1, polyhedron2).isColliding());
    }

    @Test
    void testCollidingPolyhedra() {
        ConvexPolyhedron polyhedron1 = createCollidingPolyhedron1();
        ConvexPolyhedron polyhedron2 = createCollidingPolyhedron2();

        assertTrue(gjk.isCollide(polyhedron1, polyhedron2).isColliding());
    }

    private ConvexPolyhedron createNonCollidingPolyhedron1() {
        List<Vector3f> vertices = new ArrayList<>();
        vertices.add(new Vector3f(0, 0, 0));
        vertices.add(new Vector3f(1, 0, 0));
        vertices.add(new Vector3f(1, 1, 0));
        vertices.add(new Vector3f(0, 1, 0));

        return new ConvexPolyhedron(vertices);
    }

    private ConvexPolyhedron createNonCollidingPolyhedron2() {
        List<Vector3f> vertices = new ArrayList<>();
        vertices.add(new Vector3f(2, 2, 0));
        vertices.add(new Vector3f(3, 2, 0));
        vertices.add(new Vector3f(3, 3, 0));
        vertices.add(new Vector3f(2, 3, 0));

        return new ConvexPolyhedron(vertices);
    }

    private ConvexPolyhedron createCollidingPolyhedron1() {
        List<Vector3f> vertices = new ArrayList<>();
        vertices.add(new Vector3f(0, 0, 0));
        vertices.add(new Vector3f(1, 0, 0));
        vertices.add(new Vector3f(1, 1, 0));
        vertices.add(new Vector3f(0, 1, 0));

        return new ConvexPolyhedron(vertices);
    }

    private ConvexPolyhedron createCollidingPolyhedron2() {
        List<Vector3f> vertices = new ArrayList<>();
        vertices.add(new Vector3f(0.5f, 0.5f, 0));
        vertices.add(new Vector3f(1.5f, 0.5f, 0));
        vertices.add(new Vector3f(1.5f, 1.5f, 0));
        vertices.add(new Vector3f(0.5f, 1.5f, 0));

        return new ConvexPolyhedron(vertices);
    }


    @Test
    public void testCollidingOBBs() {
        OBB obb1 = new OBB(
                new Vector3f(0.0f, 3.0f, 0.0f),
                new Vector3f(2.0f, 2.0f, 2.0f)
        );

        OBB obb2 = new OBB(
                new Vector3f(0.0f, 4.0f, 0.0f),
                new Vector3f(2.0f, 2.0f, 2.0f)
        );

        assertTrue(gjk.isCollide(obb1, obb2).isColliding(), "OBBs should be colliding");
    }

    @Test
    public void testNonCollidingOBBs() {
        Vector3f center1 = new Vector3f(0.0f, 0.0f, 0.0f);
        Vector3f halfExtents1 = new Vector3f(1.0f, 1.0f, 1.0f);
        OBB obb1 = new OBB(center1, halfExtents1);

        Vector3f center2 = new Vector3f(3.0f, 3.0f, 3.0f);
        Vector3f halfExtents2 = new Vector3f(1.0f, 1.0f, 1.0f);
        OBB obb2 = new OBB(center2, halfExtents2);

        boolean collision = gjk.isCollide(obb1, obb2).isColliding();

        assertFalse(collision);
    }

    @Test
    public void testNonParallelEdgesOBB() {
        // Create two non-parallel OBBs
        OBB obb1 = new OBB(new Vector3f(0, 0, 0), new Vector3f(1, 1, 1));
        OBB obb2 = new OBB(new Vector3f(4, 4, 4), new Vector3f(1, 1, 1));

        assertFalse(gjk.isCollide(obb1, obb2).isColliding(), "Non-parallel OBBs should not be colliding");
    }

    @Test
    public void testIntersectsTriangle() {
        // Create two colliding triangles
        Triangle triangle1 = new Triangle(
                new Vector3f(0, 0, 0),
                new Vector3f(1, 0, 0),
                new Vector3f(0, 1, 0)
        );
        Triangle triangle2 = new Triangle(
                new Vector3f(0.5f, 0.5f, 0),
                new Vector3f(1.5f, 0.5f, 0),
                new Vector3f(0.5f, 1.5f, 0)
        );
        assertTrue(gjk.isCollide(triangle1, triangle2).isColliding(), "Triangles should be colliding");
    }

    @Test
    public void testDoesNotIntersectTriangle() {
        // Create two non-colliding triangles
        Triangle triangle1 = new Triangle(
                new Vector3f(0, 0, 1),
                new Vector3f(1, 0, 0),
                new Vector3f(0, 1, 0)
        );
        Triangle triangle2 = new Triangle(
                new Vector3f(2, 2, 0),
                new Vector3f(3, 2, 2),
                new Vector3f(2, 3, 0)
        );

        assertFalse(gjk.isCollide(triangle1, triangle2).isColliding(), "Triangles should not be colliding");
    }

    @Test
    public void parallelEdgesTriangle() {
        Triangle triangle1 = new Triangle(
                new Vector3f(-2, -1, 0),
                new Vector3f(-3, 0, 0),
                new Vector3f(-1, 0, 0)
        );

        Triangle triangle2 = new Triangle(
                new Vector3f(2, 1, 0),
                new Vector3f(3, 0, 0),
                new Vector3f(1, 0, 0)
        );

        assertFalse(gjk.isCollide(triangle1, triangle2).isColliding(), "Triangles not be colliding");
    }

    @Test
    void testIsAABBCollidingWithAABB() {
        Vector3f min1 = new Vector3f(0, 0, 0);
        Vector3f max1 = new Vector3f(2, 2, 2);
        AABB aabb1 = new AABB(min1, max1);

        Vector3f min2 = new Vector3f(1, 1, 1);
        Vector3f max2 = new Vector3f(3, 3, 3);
        AABB aabb2 = new AABB(min2, max2);

        boolean result = gjk.isCollide(aabb1, aabb2).isColliding();

        assertTrue(result, "AABBs should be colliding");

    }

    @Test
    void testIsAABBNoCollidingWithAABB() {
        Vector3f min1 = new Vector3f(0, 0, 0);
        Vector3f max1 = new Vector3f(2, 2, 2);
        AABB aabb1 = new AABB(min1, max1);

        Vector3f min2 = new Vector3f(3, 3, 3);
        Vector3f max2 = new Vector3f(5, 5, 5);
        AABB aabb2 = new AABB(min2, max2);

        boolean result = gjk.isCollide(aabb1, aabb2).isColliding();

        assertFalse(result, "AABBs should not be colliding");
    }

    @Test
    void testIsSphereCollidingWithSphere() {
        Sphere sphere1 = new Sphere(new Vector3f(0, 0, 0), 1.0f);
        Sphere sphere2 = new Sphere(new Vector3f(1, 1, 1), 1.0f);

        boolean result = gjk.isCollide(sphere1, sphere2).isColliding();

        assertTrue(result, "Spheres should be colliding");
    }

    @Test
    void testIsSphereNoCollidingWithSphere() {
        Sphere sphere1 = new Sphere(new Vector3f(0, 0, 0), 1.0f);
        Sphere sphere2 = new Sphere(new Vector3f(3, 3, 3), 1.0f);

        boolean result = gjk.isCollide(sphere1, sphere2).isColliding();

        assertFalse(result, "Spheres should not be colliding");
    }

    @Test
    void testOBBCollidingWithSphere() {
        Vector3f center1 = new Vector3f(0.0f, 0.0f, 0.0f);
        Vector3f halfExtents1 = new Vector3f(1.0f, 1.0f, 1.0f);
        OBB obb = new OBB(center1, halfExtents1);

        Sphere sphere = new Sphere(new Vector3f(0, 1.5f, 0), 1.0f);

        boolean result = gjk.isCollide(sphere, obb).isColliding();

        assertTrue(result, "OBB Sphere should be colliding");
    }

    @Test
    void testOBBNotCollidingWithSphere() {
        Vector3f center1 = new Vector3f(8.0f, 8.0f, 8.0f);
        Vector3f halfExtents1 = new Vector3f(1.0f, 1.0f, 1.0f);
        OBB obb = new OBB(center1, halfExtents1);


        Sphere sphere = new Sphere(new Vector3f(2, 2, 2), 0.5f);
        boolean result = gjk.isCollide(sphere, obb).isColliding();

        assertFalse(result, "OBB Sphere should not be colliding");
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
        boolean result = gjk.isCollide(sphere, triangle).isColliding();
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

        boolean result = gjk.isCollide(sphere, triangle).isColliding();

        assertFalse(result, "Sphere should not be colliding with the triangle");
    }

    @Test
    void testAABBCollidingWithSphere() {
        Vector3f min = new Vector3f(-1, -1, -1);
        Vector3f max = new Vector3f(1, 1, 1);
        AABB aabb = new AABB(min, max);

        Sphere sphere = new Sphere(new Vector3f(0, 1.5f, 0), 1.0f);

        boolean result = gjk.isCollide(sphere, aabb).isColliding();

        assertTrue(result, "AABB Sphere should be colliding");
    }

    @Test
    void testAABBNotCollidingWithSphere() {
        Vector3f min = new Vector3f(-1, -1, -1);
        Vector3f max = new Vector3f(1, 1, 1);
        AABB aabb = new AABB(min, max);

        Sphere sphere = new Sphere(new Vector3f(2, 2, 2), 0.5f);
        boolean result = gjk.isCollide(sphere, aabb).isColliding();

        assertFalse(result, "AABB Sphere should not be colliding");
    }

    @Test
    public void testIsCapsuleColliding() {
        Capsule capsule3 = new Capsule(new Vector3f(0, 0, 0), new Vector3f(0, 3, 0), 1.0f);
        Capsule capsule4 = new Capsule(new Vector3f(0, 2, 0), new Vector3f(0, 5, 0), 1.0f);

        boolean result = gjk.isCollide(capsule3, capsule4).isColliding();
        // Assert that the capsules are not colliding
        assertTrue(result, "Capsules should not be colliding");
    }

    @Test
    public void testIsCapsuleNotColliding() {
        Capsule capsule1 = new Capsule(new Vector3f(0, 0, 0), new Vector3f(0, 3, 0), 1.0f);
        Capsule capsule2 = new Capsule(new Vector3f(0, 5, 0), new Vector3f(0, 8, 0), 1.0f);

        boolean result = gjk.isCollide(capsule1, capsule2).isColliding();

        assertFalse(result, "Capsules not be should be colliding");
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

        assertTrue(gjk.isCollide(capsule, triangle).isColliding());
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

        assertFalse(gjk.isCollide(capsule, triangle).isColliding());
    }

    @Test
    public void testCapsuleCollidingWithOBB() {
        // Create a colliding capsule and OBB
        Capsule capsule = new Capsule(new Vector3f(0, 0, 0), new Vector3f(0, 1, 0), 0.5f);
        OBB obb = new OBB(new Vector3f(0, 0.5f, 0), new Vector3f(0.5f, 0.5f, 0.5f));

        assertTrue(gjk.isCollide(capsule, obb).isColliding());
    }

    @Test
    public void testCapsuleNotCollidingWithOBB() {
        // Create a non-colliding capsule and OBB
        Capsule capsule = new Capsule(new Vector3f(3, 3, 0), new Vector3f(5, 5, 0), 0.5f);
        OBB obb = new OBB(new Vector3f(0, 0, 0), new Vector3f(0.5f, 0.5f, 0.5f));

        assertFalse(gjk.isCollide(capsule, obb).isColliding());
    }

    @Test
    void testCapsuleCollidingWithAABB() {
        Capsule capsule = new Capsule(new Vector3f(0, 0, 0), new Vector3f(0, 2, 0), 1.0f);
        AABB aabb = new AABB(new Vector3f(-1, -1, -1), new Vector3f(1, 1, 1));

        boolean result = gjk.isCollide(capsule, aabb).isColliding();
        assertTrue(result, "Capsule and AABB should be colliding");
    }

    @Test
    void testCapsuleNotCollidingWithAABB() {
        Capsule capsule = new Capsule(new Vector3f(1, 1, 3), new Vector3f(1, 2, 3), 1.0f);
        AABB aabb = new AABB(new Vector3f(-1, -1, -1), new Vector3f(1, 1, 1));

        boolean result = gjk.isCollide(capsule, aabb).isColliding();
        assertFalse(result, "Capsule and AABB should not be colliding");
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

        boolean collision = gjk.isCollide(sphere, capsule).isColliding();

        assertTrue(collision, "Sphere Capsule should be colliding");
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

        boolean collision = gjk.isCollide(sphere, capsule).isColliding();

        assertFalse(collision, "Sphere Capsule should not be colliding");
    }

    @Test
    public void testCollidingCylinders() {
        Cylinder cylinder1 = new Cylinder(new Vector3f(1.0f, 1.0f, 1.0f), 1.0f, 2.0f);
        Cylinder cylinder2 = new Cylinder(new Vector3f(2.0f, 2.0f, 2.0f), 1.0f, 2.0f);
        assertTrue(gjk.isCollide(cylinder1, cylinder2).isColliding());
    }

    @Test
    public void testNonCollidingCylinders() {
        Cylinder cylinder1 = new Cylinder(new Vector3f(1.0f, 1.0f, 1.0f), 1.0f, 2.0f);
        Cylinder cylinder2 = new Cylinder(new Vector3f(5.0f, 5.0f, 5.0f), 1.0f, 2.0f);
        assertFalse(gjk.isCollide(cylinder1, cylinder2).isColliding());
    }

    @Test
    void testCylinderCollidingWithOBB() {
        Cylinder cylinder = new Cylinder(new Vector3f(0, 0, 0), 1.0f, 2.0f);
        OBB obb = new OBB(new Vector3f(1, 0, 0), new Vector3f(1, 1, 1));

        boolean result = gjk.isCollide(cylinder, obb).isColliding();
        assertTrue(result, "Collision should be detected");
    }

    @Test
    void testCylinderNotCollidingWithOBB() {
        Cylinder cylinder = new Cylinder(new Vector3f(0, 0, 0), 1.0f, 2.0f);
        OBB obb = new OBB(new Vector3f(4, 4, 4), new Vector3f(1, 1, 1));

        boolean result = gjk.isCollide(cylinder, obb).isColliding();
        assertFalse(result, "No collision should be detected");
    }

    @Test
    void testCylinderCollidingWithTriangle() {
        Cylinder cylinder = new Cylinder(new Vector3f(0, 0, 0), 1.0f, 2.0f);

        Triangle triangle = new Triangle(new Vector3f(0, 0, 0), new Vector3f(1, 0, 0), new Vector3f(0, 1, 0));

        boolean collision = gjk.isCollide(cylinder, triangle).isColliding();
        assertTrue(collision, "Expected collision between cylinder and triangle");
    }

    @Test
    void testCylinderNotCollidingWithTriangle() {
        Cylinder cylinder = new Cylinder(new Vector3f(5, 5, 5), 1.0f, 2.0f);

        Triangle triangle = new Triangle(new Vector3f(-3, 0, 0), new Vector3f(-2, 0, 0), new Vector3f(-2, -1, 0));

        boolean collision = gjk.isCollide(cylinder, triangle).isColliding();
        assertFalse(collision, "Expected no collision between cylinder and triangle");
    }

    @Test
    void testCylinderCollidingWithAABB() {
        Cylinder cylinder = new Cylinder(new Vector3f(0, 1, 0), 1.0f, 4.0f);
        AABB aabb = new AABB(new Vector3f(-1, 0, -1), new Vector3f(1, 3, 1));

        boolean result = gjk.isCollide(cylinder, aabb).isColliding();
        assertTrue(result, "Cylinder and AABB should be colliding");
    }

    @Test
    void testCylinderNotCollidingWithAABB() {
        Cylinder cylinder = new Cylinder(new Vector3f(5, 0, 0), 1.0f, 2.0f);
        AABB aabb = new AABB(new Vector3f(-1, 0, -1), new Vector3f(1, 3, 1));

        boolean result = gjk.isCollide(cylinder, aabb).isColliding();
        assertFalse(result, "Cylinder and AABB should not be colliding");
    }

    @Test
    void testCapsuleCollidingWithCylinder() {
        Capsule capsule = new Capsule(new Vector3f(0, 1, 0), new Vector3f(0, 3, 0), 0.5f);
        Cylinder cylinder = new Cylinder(new Vector3f(0, 2, 0), 1.0f, 4.0f);

        boolean result = gjk.isCollide(capsule, cylinder).isColliding();
        assertTrue(result, "Capsule and Cylinder should be colliding");
    }

    @Test
    void testCapsuleNotCollidingWithCylinder() {
        Capsule capsule = new Capsule(new Vector3f(0, 10, 0), new Vector3f(0, 12, 0), 1f);
        Cylinder cylinder = new Cylinder(new Vector3f(0, 2, 0), 1.0f, 4.0f);

        boolean result = gjk.isCollide(capsule, cylinder).isColliding();
        assertFalse(result, "Capsule and Cylinder should not be colliding");
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

        boolean collision = gjk.isCollide(sphere, cylinder).isColliding();

        assertTrue(collision, "Sphere Cylinder should be colliding");
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

        boolean collision = gjk.isCollide(sphere, cylinder).isColliding();

        assertFalse(collision, "Sphere Cylinder should not be colliding");
    }

    @Test
    public void testLineCollision() {
        // Test cases where the lines intersect
        Line line1 = new Line(new Vector3f(0, 0, 0), new Vector3f(1, 1, 0));
        Line line2 = new Line(new Vector3f(0, 1, 0), new Vector3f(1, 0, 0));
        assertTrue(gjk.isCollide(line1, line2).isColliding());

        Line line3 = new Line(new Vector3f(0, 0, 0), new Vector3f(1, 1, 0));
        Line line4 = new Line(new Vector3f(0.5f, 0.5f, 0), new Vector3f(1, 0, 0));
        assertTrue(gjk.isCollide(line3, line4).isColliding());

        // Test cases where the lines do not intersect
        Line line5 = new Line(new Vector3f(0, 0, 0), new Vector3f(1, 1, 0));
        Line line6 = new Line(new Vector3f(2, 2, 0), new Vector3f(3, 3, 0));
        assertFalse(gjk.isCollide(line5, line6).isColliding());

        Line line7 = new Line(new Vector3f(0, 0, 0), new Vector3f(1, 1, 0));
        Line line8 = new Line(new Vector3f(0, 2, 0), new Vector3f(1, 3, 0));
        assertFalse(gjk.isCollide(line7, line8).isColliding());

        // Test cases where the lines are parallel but not collinear
        Line line9 = new Line(new Vector3f(0, 0, 0), new Vector3f(1, 1, 0));
        Line line10 = new Line(new Vector3f(0, 0, 1), new Vector3f(1, 1, 1));
        assertFalse(gjk.isCollide(line9, line10).isColliding());

        Line line11 = new Line(new Vector3f(2, 2, 2), new Vector3f(4, 4, 4));
        Line line12 = new Line(new Vector3f(1, -1, 1), new Vector3f(1, 1, 1));
        assertFalse(gjk.isCollide(line11, line12).isColliding());
    }

    @Test
    public void testOBBCollisionLine() {
        // Create an OBB with a known center and half extents
        OBB obb = new OBB(new Vector3f(1, 1, 1), new Vector3f(1, 1, 1));

        // Test a line that intersects with the OBB
        Line intersectingLine = new Line(new Vector3f(0, 0, 0), new Vector3f(2, 2, 2));
        assertTrue(gjk.isCollide(intersectingLine, obb).isColliding());

        // Test a line that does not intersect with the OBB
        Line nonIntersectingLine = new Line(new Vector3f(3, 3, 0), new Vector3f(5, 5, 0));
        assertFalse(gjk.isCollide(nonIntersectingLine, obb).isColliding());

        // Create an OBB
        OBB obb2 = new OBB(new Vector3f(0.0f, 0.0f, 0.0f), new Vector3f(1.0f, 1.0f, 1.0f));

        // Create a line that intersects with the OBB
        Line line1 = new Line(new Vector3f(-2.0f, 0.0f, 0.0f), new Vector3f(2.0f, 0.0f, 0.0f));

        // Create a line that doesn't intersect with the OBB
        Line line2 = new Line(new Vector3f(-2.0f, 2.0f, 0.0f), new Vector3f(-1.0f, 2.0f, 0.0f));

        assertTrue(gjk.isCollide(line1, obb2).isColliding()); // Expect collision
        assertFalse(gjk.isCollide(line2, obb2).isColliding()); // Expect no collision
    }

    @Test
    public void testTriangleCollisionLine() {
        // Create a simple triangle and line segment for testing
        Vector3f vertex1 = new Vector3f(0, 0, 0);
        Vector3f vertex2 = new Vector3f(2, 0, 0);
        Vector3f vertex3 = new Vector3f(2, 2, 0);

        Triangle triangle = new Triangle(vertex1, vertex2, vertex3);
        Line line = new Line(new Vector3f(-1f, 0, 0), new Vector3f(3f, 0, 0));

        // Test if the line and triangle intersect
        assertTrue(gjk.isCollide(line, triangle).isColliding());

        // Test non-intersecting case
        Line nonIntersectingLine = new Line(new Vector3f(3, 3, 3), new Vector3f(4, 4, 4));
        assertFalse(gjk.isCollide(nonIntersectingLine, triangle).isColliding());

        Line line2 = new Line(new Vector3f(0.5f, 0.5f, 0), new Vector3f(1.5f, 1.5f, 0));

        assertTrue(gjk.isCollide(line2, triangle).isColliding());

        // Create a triangle entirely inside a line segment
        vertex1 = new Vector3f(0, 0, 0);
        vertex2 = new Vector3f(4, 0, 0);
        vertex3 = new Vector3f(4, 4, 0);

        Triangle triangle2 = new Triangle(vertex1, vertex2, vertex3);
        Line line3 = new Line(new Vector3f(1, 1, 0), new Vector3f(5, 5, 0));

        assertTrue(gjk.isCollide(line3, triangle2).isColliding());
    }

    @Test
    public void testAABBCollisionLine() {
        // Create an AABB with minimum corner (-1, -1, -1) and maximum corner (1, 1, 1)
        AABB aabb = new AABB(new Vector3f(-1, -1, -1), new Vector3f(1, 1, 1));

        // Test cases where the line intersects the AABB
        Line line1 = new Line(new Vector3f(0, 0, -2), new Vector3f(0, 0, 2));
        assertTrue(gjk.isCollide(line1, aabb).isColliding());

        Line line2 = new Line(new Vector3f(-2, -2, -2), new Vector3f(-1, -1, -1));
        assertTrue(gjk.isCollide(line2, aabb).isColliding());

        Line line3 = new Line(new Vector3f(0, 0, 0), new Vector3f(0, 0, 1));
        assertTrue(gjk.isCollide(line3, aabb).isColliding());

        // Test cases where the line does not intersect the AABB
        Line line4 = new Line(new Vector3f(2, 2, 2), new Vector3f(3, 3, 3));
        assertFalse(gjk.isCollide(line4, aabb).isColliding());

        Line line5 = new Line(new Vector3f(2, 2, 2), new Vector3f(2, 2, 3));
        assertFalse(gjk.isCollide(line5, aabb).isColliding());

        Line line6 = new Line(new Vector3f(2, 0, 0), new Vector3f(3, 0, 0));
        assertFalse(gjk.isCollide(line6, aabb).isColliding());
    }

    @Test
    public void testCapsuleCollisionLine() {
        // Create a capsule with start point (-1, 0, 0), end point (1, 0, 0), and radius 0.5
        Capsule capsule = new Capsule(new Vector3f(-1, 0, 0), new Vector3f(1, 0, 0), 0.5f);

        // Test cases where the line collides with the capsule
        Line line1 = new Line(new Vector3f(0, 0, -1), new Vector3f(0, 0, 1));
        assertTrue(gjk.isCollide(line1, capsule).isColliding());

        Line line2 = new Line(new Vector3f(0, 0, 0), new Vector3f(0, 0, 1));
        assertTrue(gjk.isCollide(line2, capsule).isColliding());

        Line line3 = new Line(new Vector3f(-2, 0, 0), new Vector3f(-0.5f, 0, 0));
        assertTrue(gjk.isCollide(line3, capsule).isColliding());

        Line line4 = new Line(new Vector3f(0, 0, 2), new Vector3f(0, 0, 3));
        assertFalse(gjk.isCollide(line4, capsule).isColliding());

        Line line5 = new Line(new Vector3f(2, 0, 0), new Vector3f(3, 0, 0));
        assertFalse(gjk.isCollide(line5, capsule).isColliding());

        Line line6 = new Line(new Vector3f(0, 0, -2), new Vector3f(0, 0, -1));
        assertFalse(gjk.isCollide(line6, capsule).isColliding());
    }


    @Test
    public void testConvexPolyhedronCollisionLine() {
        // Create a convex polyhedron with vertices defining a simple cube
        List<Vector3f> cubeVertices = new ArrayList<>();
        cubeVertices.add(new Vector3f(-1, -1, -1));
        cubeVertices.add(new Vector3f(1, -1, -1));
        cubeVertices.add(new Vector3f(1, 1, -1));
        cubeVertices.add(new Vector3f(-1, 1, -1));
        cubeVertices.add(new Vector3f(-1, -1, 1));
        cubeVertices.add(new Vector3f(1, -1, 1));
        cubeVertices.add(new Vector3f(1, 1, 1));
        cubeVertices.add(new Vector3f(-1, 1, 1));

        ConvexPolyhedron cube = new ConvexPolyhedron(cubeVertices);

        // Test cases where the line collides with the cube
        Line line1 = new Line(new Vector3f(0, 0, -2), new Vector3f(0, 0, 2));
        assertTrue(gjk.isCollide(line1, cube).isColliding());

        Line line2 = new Line(new Vector3f(0, 0, 0), new Vector3f(0, 0, 1));
        assertTrue(gjk.isCollide(line2, cube).isColliding());

        Line line3 = new Line(new Vector3f(-2, -2, -2), new Vector3f(1, 1, 1));
        assertTrue(gjk.isCollide(line3, cube).isColliding());

        // Test cases where the line does not collide with the cube
        Line line4 = new Line(new Vector3f(2, 2, 2), new Vector3f(4, 4, 4));
        assertFalse(gjk.isCollide(line4, cube).isColliding());

        Line line5 = new Line(new Vector3f(2, 2, 2), new Vector3f(3, 3, 3));
        assertFalse(gjk.isCollide(line5, cube).isColliding());

        Line line6 = new Line(new Vector3f(2, 2, 2), new Vector3f(2, 2, 3));
        assertFalse(gjk.isCollide(line6, cube).isColliding());
    }

    @Test
    public void testSphereCollisionLine() {
        // Create a sphere with center (0, 0, 0) and radius 1.
        Sphere sphere = new Sphere(new Vector3f(0, 0, 0), 1);

        // Test a line that collides with the sphere.
        Line line1 = new Line(new Vector3f(2, 0, 0), new Vector3f(-2, 0, 0));
        assertTrue(gjk.isCollide(line1, sphere).isColliding());

        // Test a line that does not collide with the sphere.
        Line line2 = new Line(new Vector3f(2, 2, 0), new Vector3f(3, 3, 0));
        assertFalse(gjk.isCollide(line2, sphere).isColliding());

        // Test a line that starts inside the sphere.
        Line line3 = new Line(new Vector3f(0, 0, 0), new Vector3f(1, 1, 1));
        assertTrue(gjk.isCollide(line3, sphere).isColliding());

        // Test a line that ends inside the sphere.
        Line line4 = new Line(new Vector3f(2, 2, 2), new Vector3f(0, 0, 0));
        assertTrue(gjk.isCollide(line4, sphere).isColliding());

        // Test a line that is completely outside the sphere.
        Line line5 = new Line(new Vector3f(3, 3, 3), new Vector3f(4, 4, 4));
        assertFalse(gjk.isCollide(line5, sphere).isColliding());
    }

    @Test
    public void testCylinderCollisionLine() {
        // Create a cylinder with center (0, 0, 0), radius 1, and height 2.
        Cylinder cylinder = new Cylinder(new Vector3f(0, 0, 0), 1, 2);

        // Test a line that collides with the cylinder.
        Line line1 = new Line(new Vector3f(0, 1, 0), new Vector3f(0, -1, 0));
        assertTrue(gjk.isCollide(line1, cylinder).isColliding());

        // Test a line that does not collide with the cylinder.
        Line line2 = new Line(new Vector3f(2, 2, 2), new Vector3f(3, 3, 3));
        assertFalse(gjk.isCollide(line2, cylinder).isColliding());

        // Test a line that starts inside the cylinder.
        Line line3 = new Line(new Vector3f(0, 0, 0), new Vector3f(0, 1, 0));
        assertTrue(gjk.isCollide(line3, cylinder).isColliding());

        // Test a line that ends inside the cylinder.
        Line line4 = new Line(new Vector3f(2, 2, 0), new Vector3f(0, 0, 0));
        assertTrue(gjk.isCollide(line4, cylinder).isColliding());

        // Test a line that is completely outside the cylinder.
        Line line5 = new Line(new Vector3f(2, 2, 2), new Vector3f(4, 4, 4));
        assertFalse(gjk.isCollide(line5, cylinder).isColliding());
    }
}