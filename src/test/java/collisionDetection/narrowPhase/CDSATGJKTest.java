package collisionDetection.narrowPhase;

import collisionDetection.narrowPhase.cd.CDSATGJK;
import collisionDetection.primitive.*;
import collisionDetection.primitive.terrain.TerrainShape;
import math.Vector3f;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class CDSATGJKTest {


    @Test
    public void testCapsuleCollisionLine() {
        // Create a capsule with start point (-1, 0, 0), end point (1, 0, 0), and radius 0.5
        Capsule capsule = new Capsule(new Vector3f(-1, 0, 0), new Vector3f(1, 0, 0), 0.5f);

        // Test cases where the line collides with the capsule
        Line line1 = new Line(new Vector3f(0, 0, -1), new Vector3f(0, 0, 1));
        assertTrue(CDSATGJK.isCollide(line1, capsule));

        Line line2 = new Line(new Vector3f(0, 0, 0), new Vector3f(0, 0, 1));
        assertTrue(CDSATGJK.isCollide(line2, capsule));

        Line line3 = new Line(new Vector3f(-2, 0, 0), new Vector3f(-1, 0, 0));
        assertTrue(CDSATGJK.isCollide(line3, capsule));

        Line line4 = new Line(new Vector3f(0, 0, 2), new Vector3f(0, 0, 3));
        assertFalse(CDSATGJK.isCollide(line4, capsule));

        Line line5 = new Line(new Vector3f(2, 0, 0), new Vector3f(3, 0, 0));
        assertFalse(CDSATGJK.isCollide(line5, capsule));

        Line line6 = new Line(new Vector3f(0, 0, -2), new Vector3f(0, 0, -1));
        assertFalse(CDSATGJK.isCollide(line6, capsule));
    }




    @Test
    public void testSphereCollisionLine() {
        // Create a sphere with center (0, 0, 0) and radius 1.
        Sphere sphere = new Sphere(new Vector3f(0, 0, 0), 1);

        // Test a line that collides with the sphere.
        Line line1 = new Line(new Vector3f(2, 0, 0), new Vector3f(-2, 0, 0));
        assertTrue(CDSATGJK.isCollide(line1, sphere));

        // Test a line that does not collide with the sphere.
        Line line2 = new Line(new Vector3f(2, 2, 0), new Vector3f(3, 3, 0));
        assertFalse(CDSATGJK.isCollide(line2, sphere));

        // Test a line that starts inside the sphere.
        Line line3 = new Line(new Vector3f(0, 0, 0), new Vector3f(1, 1, 1));
        assertTrue(CDSATGJK.isCollide(line3, sphere));

        // Test a line that ends inside the sphere.
        Line line4 = new Line(new Vector3f(2, 2, 2), new Vector3f(0, 0, 0));
        assertTrue(CDSATGJK.isCollide(line4, sphere));

        // Test a line that is completely outside the sphere.
        Line line5 = new Line(new Vector3f(3, 3, 3), new Vector3f(4, 4, 4));
        assertFalse(CDSATGJK.isCollide(line5, sphere));
    }

    @Test
    public void testCylinderCollisionLine() {
        // Create a cylinder with center (0, 0, 0), radius 1, and height 2.
        Cylinder cylinder = new Cylinder(new Vector3f(0, 0, 0), 1, 2);

        // Test a line that collides with the cylinder.
        Line line1 = new Line(new Vector3f(0, 1, 0), new Vector3f(0, -1, 0));
        assertTrue(CDSATGJK.isCollide(line1, cylinder));

        // Test a line that does not collide with the cylinder.
        Line line2 = new Line(new Vector3f(2, 2, 2), new Vector3f(3, 3, 3));
        assertFalse(CDSATGJK.isCollide(line2, cylinder));

        // Test a line that starts inside the cylinder.
        Line line3 = new Line(new Vector3f(0, 0, 0), new Vector3f(0, 1, 0));
        assertTrue(CDSATGJK.isCollide(line3, cylinder));

        // Test a line that ends inside the cylinder.
        Line line4 = new Line(new Vector3f(2, 2, 0), new Vector3f(0, 0, 0));
        assertTrue(CDSATGJK.isCollide(line4, cylinder));

        // Test a line that is completely outside the cylinder.
        Line line5 = new Line(new Vector3f(2, 2, 2), new Vector3f(4, 4, 4));
        assertFalse(CDSATGJK.isCollide(line5, cylinder));
    }

    @Test
    public void testPlaneCollisionLine() {
        // Create a plane with a normal vector (0, 1, 0) and a distance of 2 units from the origin.
        Plane plane = new Plane(new Vector3f(0, 1, 0), 2);

        // Test a line that collides with the plane.
        Line line1 = new Line(new Vector3f(0, 0, 0), new Vector3f(0, 3, 0));
        assertTrue(CDSATGJK.isCollide(line1, plane));

        // Test a line that does collide with the plane.
        Line line2 = new Line(new Vector3f(1, 1, 1), new Vector3f(2, 2, 2));
        assertTrue(CDSATGJK.isCollide(line2, plane));

        // Test a line that is parallel to the plane (no collision).
        Line line3 = new Line(new Vector3f(0, 0, 0), new Vector3f(1, 0, 1));
        assertFalse(CDSATGJK.isCollide(line3, plane));

        // Test a line that intersects the plane at its endpoint.
        Line line4 = new Line(new Vector3f(0, 2, 0), new Vector3f(0, 3, 0));
        assertTrue(CDSATGJK.isCollide(line4, plane));

        // Create a plane with a known normal and distance
        Plane plane2 = new Plane(new Vector3f(0, 0, 1), 0);

        // Test a line that intersects with the plane
        Line intersectingLine = new Line(new Vector3f(0, 0, -1), new Vector3f(0, 0, 1));
        assertTrue(CDSATGJK.isCollide(intersectingLine, plane2));

        // Test a line that is parallel to the plane and does not intersect
        Line parallelLine = new Line(new Vector3f(1, 1, 1), new Vector3f(2, 2, 2));
        assertFalse(CDSATGJK.isCollide(parallelLine, plane2));
    }

    @Test
    public void testTerrainCollisionLine() {
        // Create a TerrainShape with height data, borders, width, and length for testing collision
        float[][] heightData = new float[5][5];

        // Define a simple AABB for terrain borders
        AABB borders = new AABB(new Vector3f(-1, -1, -1), new Vector3f(5, 5, 5));

        // Initialize the TerrainShape object
        TerrainShape terrainShape = new TerrainShape(heightData, borders, new Vector3f(1,1,1));

        // Create a Line object representing a line segment that collides with the terrain
        Line line = new Line(new Vector3f(0, 5, 0), new Vector3f(0, -1, 0));

        // Assert that the result is true since the line should collide with the terrain
        assertTrue(CDSATGJK.isCollide(line, terrainShape));

        // Create a Line object representing a line segment that does not collide with the terrain
        Line line2 = new Line(new Vector3f(3, 3, 3), new Vector3f(4, 4, 4));

        // Assert that the result is false since the line should not collide with the terrain
        assertFalse(CDSATGJK.isCollide(line2, terrainShape));
    }

    @Test
    void testAABBCollidingWithSphere() {
        Vector3f min = new Vector3f(-1, -1, -1);
        Vector3f max = new Vector3f(1, 1, 1);
        AABB aabb = new AABB(min, max);

        Sphere sphere = new Sphere(new Vector3f(0, 1.5f, 0), 1.0f);

        boolean result = CDSATGJK.isCollide(sphere, aabb);

        assertTrue(result, "AABB Sphere should be colliding");
    }

    @Test
    void testAABBNotCollidingWithSphere() {
        Vector3f min = new Vector3f(-1, -1, -1);
        Vector3f max = new Vector3f(1, 1, 1);
        AABB aabb = new AABB(min, max);

        Sphere sphere = new Sphere(new Vector3f(2, 2, 2), 0.5f);
        boolean result = CDSATGJK.isCollide(sphere, aabb);

        assertFalse(result, "AABB Sphere should not be colliding");
    }


    @Test
    public void testSphereCollidingWithPlane() {
        Sphere sphere = new Sphere(new Vector3f(0.0f, 4.0f, 0.0f), 2.0f);
        Plane plane = new Plane(new Vector3f(0.0f, 1.0f, 0.0f), 3.0f);

        boolean result = CDSATGJK.isCollide(sphere, plane);

        assertTrue(result,"Sphere Plane should be colliding");
    }

    @Test
    public void testSphereNotCollidingWithPlane() {
        Sphere sphere = new Sphere(new Vector3f(0.0f, 5.0f, 0.0f), 2.0f);
        Plane plane = new Plane(new Vector3f(1.0f, 0.0f, 0.0f), 10.0f);

        boolean result = CDSATGJK.isCollide(sphere, plane);

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
        boolean result = CDSATGJK.isCollide(sphere, triangle);
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

        boolean result = CDSATGJK.isCollide(sphere, triangle);

        assertFalse(result, "Sphere should not be colliding with the triangle");
    }

    @Test
    void testOBBCollidingWithSphere() {
        Vector3f center1 = new Vector3f(0.0f, 0.0f, 0.0f);
        Vector3f halfExtents1 = new Vector3f(1.0f, 1.0f, 1.0f);
        OBB obb = new OBB(center1, halfExtents1);

        Sphere sphere = new Sphere(new Vector3f(0, 1.5f, 0), 1.0f);

        boolean result = CDSATGJK.isCollide(sphere, obb);

        assertTrue(result, "OBB Sphere should be colliding");
    }

    @Test
    void testOBBNotCollidingWithSphere() {
        Vector3f center1 = new Vector3f(8.0f, 8.0f, 8.0f);
        Vector3f halfExtents1 = new Vector3f(1.0f, 1.0f, 1.0f);
        OBB obb = new OBB(center1, halfExtents1);


        Sphere sphere = new Sphere(new Vector3f(2, 2, 2), 0.5f);
        boolean result = CDSATGJK.isCollide(sphere, obb);

        assertFalse(result, "OBB Sphere should not be colliding");
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

        boolean collision = CDSATGJK.isCollide(sphere, capsule);

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

        boolean collision = CDSATGJK.isCollide(sphere, capsule);

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

        boolean collision = CDSATGJK.isCollide(sphere, cylinder);

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

        boolean collision = CDSATGJK.isCollide(sphere, cylinder);

        assertFalse(collision,"Sphere Cylinder should not be colliding");
    }

    @Test
    void testPlaneCollidingWithAABB() {
        Plane plane = new Plane(new Vector3f(0, 1, 0), 0); // Plane with normal (0, 1, 0) and distance 0
        AABB aabb = new AABB(new Vector3f(-1, -1, -1), new Vector3f(1, 1, 1)); // AABB centered at (0, 0, 0)

        boolean result = CDSATGJK.isCollide(plane, aabb);
        assertTrue(result, "AABB and plane should be colliding");
    }

    @Test
    void testPlaneNotCollidingWithAABB() {
        Plane plane = new Plane(new Vector3f(0, 1, 0), 0); // Plane with normal (0, 1, 0) and distance 0
        AABB aabb = new AABB(new Vector3f(-2, -2, -2), new Vector3f(-1, -1, -1)); // AABB above the plane

        boolean result = CDSATGJK.isCollide(plane, aabb);
        assertFalse(result, "AABB and plane should not be colliding");
    }

    @Test
    void testCapsuleCollidingWithAABB() {
        Capsule capsule = new Capsule(new Vector3f(0, 0, 0), new Vector3f(0, 2, 0), 1.0f);
        AABB aabb = new AABB(new Vector3f(-1, -1, -1), new Vector3f(1, 1, 1));

        boolean result =CDSATGJK.isCollide(capsule, aabb);
        assertTrue(result, "Capsule and AABB should be colliding");
    }

    @Test
    void testCapsuleNotCollidingWithAABB() {
        Capsule capsule = new Capsule(new Vector3f(1, 1, 3), new Vector3f(1, 2, 3), 1.0f);
        AABB aabb = new AABB(new Vector3f(-1, -1, -1), new Vector3f(1, 1, 1));

        boolean result = CDSATGJK.isCollide(capsule, aabb);
        assertFalse(result, "Capsule and AABB should not be colliding");
    }

    @Test
    void testCylinderCollidingWithAABB() {
        Cylinder cylinder = new Cylinder(new Vector3f(0, 1, 0), 1.0f, 4.0f);
        AABB aabb = new AABB(new Vector3f(-1, 0, -1), new Vector3f(1, 3, 1));

        boolean result = CDSATGJK.isCollide(cylinder, aabb);
        assertTrue(result, "Cylinder and AABB should be colliding");
    }

    @Test
    void testCylinderNotCollidingWithAABB() {
        Cylinder cylinder = new Cylinder(new Vector3f(5, 0, 0), 1.0f, 2.0f);
        AABB aabb = new AABB(new Vector3f(-1, 0, -1), new Vector3f(1, 3, 1));

        boolean result = CDSATGJK.isCollide(cylinder, aabb);
        assertFalse(result, "Cylinder and AABB should not be colliding");
    }

    @Test
    void testCapsuleCollidingWithCylinder() {
        Capsule capsule = new Capsule(new Vector3f(0, 1, 0), new Vector3f(0, 3, 0), 0.5f);
        Cylinder cylinder = new Cylinder(new Vector3f(0, 2, 0), 1.0f, 4.0f);

        boolean result = CDSATGJK.isCollide(capsule, cylinder);
        assertTrue(result, "Capsule and Cylinder should be colliding");
    }

    @Test
    void testCapsuleNotCollidingWithCylinder() {
        Capsule capsule = new Capsule(new Vector3f(0, 10, 0), new Vector3f(0, 12, 0), 1f);
        Cylinder cylinder = new Cylinder(new Vector3f(0, 2, 0), 1.0f, 4.0f);

        boolean result = CDSATGJK.isCollide(capsule, cylinder);
        assertFalse(result, "Capsule and Cylinder should not be colliding");
    }

    @Test
    void testCylinderCollidingWithPlane() {
        Cylinder cylinder = new Cylinder(new Vector3f(0, 1, 0), 1.0f, 2.0f);
        Plane plane = new Plane(new Vector3f(0, 1, 0), 0); // Plane with normal (0, 1, 0) and distance 0

        boolean result = CDSATGJK.isCollide(cylinder, plane);
        assertTrue(result, "Cylinder and Plane should be colliding");
    }

    @Test
    void testCylinderNotCollidingWithPlane() {
        Cylinder cylinder = new Cylinder(new Vector3f(0, 5, 0), 1.0f, 2.0f);
        Plane plane = new Plane(new Vector3f(0, 1, 0), 0); // Plane with normal (0, 1, 0) and distance 0

        boolean result = CDSATGJK.isCollide(cylinder, plane);
        assertFalse(result, "Cylinder and Plane should not be colliding");
    }

    @Test
    void testCapsuleCollidingWithPlane() {
        Capsule capsule = new Capsule(new Vector3f(0, 1, 0), new Vector3f(0, 3, 0), 0.5f);
        Plane plane = new Plane(new Vector3f(0, 1, 0), 0); // Plane with normal (0, 1, 0) and distance 0

        boolean result = CDSATGJK.isCollide(capsule, plane);
        assertTrue(result, "Capsule and Plane should be colliding");
    }

    @Test
    void testCapsuleNotCollidingWithPlane() {
        Capsule capsule = new Capsule(new Vector3f(0, 5, 0), new Vector3f(0, 7, 0), 0.5f);
        Plane plane = new Plane(new Vector3f(0, 1, 0), 0); // Plane with normal (0, 1, 0) and distance 0

        boolean result = CDSATGJK.isCollide(capsule, plane);
        assertFalse(result, "Capsule and Plane should not be colliding");
    }

    @Test
    void testCylinderCollidingWithTriangle() {
        Cylinder cylinder = new Cylinder(new Vector3f(0, 0, 0), 1.0f, 2.0f);

        Triangle triangle = new Triangle(new Vector3f(0, 0, 0), new Vector3f(1, 0, 0), new Vector3f(0, 1, 0));

        boolean collision = CDSATGJK.isCollide(cylinder, triangle);
        assertTrue(collision, "Expected collision between cylinder and triangle");
    }

    @Test
    void testCylinderNotCollidingWithTriangle() {
        Cylinder cylinder = new Cylinder(new Vector3f(5, 5, 5), 1.0f, 2.0f);

        Triangle triangle = new Triangle(new Vector3f(-3, 0, 0), new Vector3f(-2, 0, 0), new Vector3f(-2, -1, 0));

        boolean collision = CDSATGJK.isCollide(cylinder, triangle);
        assertFalse(collision, "Expected no collision between cylinder and triangle");
    }

    @Test
    void testCylinderCollidingWithOBB() {
        Cylinder cylinder = new Cylinder(new Vector3f(0, 0, 0), 1.0f, 2.0f);
        OBB obb = new OBB(new Vector3f(1, 0, 0), new Vector3f(1, 1, 1));

        boolean result = CDSATGJK.isCollide(cylinder, obb);
        assertTrue(result, "Collision should be detected");
    }

    @Test
    void testCylinderNotCollidingWithOBB() {
        Cylinder cylinder = new Cylinder(new Vector3f(0, 0, 0), 1.0f, 2.0f);
        OBB obb = new OBB(new Vector3f(4, 4, 4), new Vector3f(1, 1, 1));

        boolean result = CDSATGJK.isCollide(cylinder, obb);
        assertFalse(result, "No collision should be detected");
    }

    @Test
    void testPlaneCollidingWithTriangle() {
        Vector3f normal = new Vector3f(0, 1, 0);
        Plane plane = new Plane(normal, 0);

        Vector3f vertex1 = new Vector3f(0, 0, 0);
        Vector3f vertex2 = new Vector3f(1, 0, 0);
        Vector3f vertex3 = new Vector3f(0.5f, 1, 0);
        Triangle triangle = new Triangle(vertex1, vertex2, vertex3);

        assertTrue(CDSATGJK.isCollide(triangle, plane));
    }

    @Test
    void testPlaneNotCollidingWithTriangle() {
        Vector3f normal = new Vector3f(0, 1, 0);
        Plane plane = new Plane(normal, 0);

        Vector3f vertex1 = new Vector3f(0, 0, 0);
        Vector3f vertex2 = new Vector3f(1, 0, 0);
        Vector3f vertex3 = new Vector3f(2, 0, 0);
        Triangle triangle = new Triangle(vertex1, vertex2, vertex3);

        assertFalse(CDSATGJK.isCollide(triangle, plane));
    }


    @Test
    public void testCapsuleCollidingWithOBB() {
        // Create a colliding capsule and OBB
        Capsule capsule = new Capsule(new Vector3f(0, 0, 0), new Vector3f(0, 1, 0), 0.5f);
        OBB obb = new OBB(new Vector3f(0, 0.5f, 0), new Vector3f(0.5f, 0.5f, 0.5f));

        assertTrue(CDSATGJK.isCollide(capsule, obb));
    }

    @Test
    public void testCapsuleNotCollidingWithOBB() {
        // Create a non-colliding capsule and OBB
        Capsule capsule = new Capsule(new Vector3f(3, 3, 0), new Vector3f(5, 5, 0), 0.5f);
        OBB obb = new OBB(new Vector3f(0, 0, 0), new Vector3f(0.5f, 0.5f, 0.5f));

        assertFalse(CDSATGJK.isCollide(capsule, obb));
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

        assertTrue(CDSATGJK.isCollide(capsule, triangle));
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

        assertFalse(CDSATGJK.isCollide(capsule, triangle));
    }

    @Test
    public void testIsCapsuleColliding() {
        Capsule capsule3 = new Capsule(new Vector3f(0, 0, 0), new Vector3f(0, 3, 0), 1.0f);
        Capsule capsule4 = new Capsule(new Vector3f(0, 2, 0), new Vector3f(0, 5, 0), 1.0f);

        boolean result = CDSATGJK.isCollide(capsule3, capsule4);
        // Assert that the capsules are not colliding
        assertTrue(result, "Capsules should not be colliding");
    }

    @Test
    public void testIsCapsuleNotColliding() {
        Capsule capsule1 = new Capsule(new Vector3f(0, 0, 0), new Vector3f(0, 3, 0), 1.0f);
        Capsule capsule2 = new Capsule(new Vector3f(2, 2, 0), new Vector3f(2, 5, 0), 1.0f);

        boolean result = CDSATGJK.isCollide(capsule1, capsule2);

        assertFalse(result, "Capsules not be should be colliding");
    }
    @Test
    public void testCollidingCylinders() {
        Cylinder cylinder1 = new Cylinder(new Vector3f(1.0f, 1.0f, 1.0f), 1.0f, 2.0f);
        Cylinder cylinder2 = new Cylinder(new Vector3f(2.0f, 2.0f, 2.0f), 1.0f, 2.0f);
        assertTrue(CDSATGJK.isCollide(cylinder1, cylinder2));
    }

    @Test
    public void testNonCollidingCylinders() {
        Cylinder cylinder1 = new Cylinder(new Vector3f(1.0f, 1.0f, 1.0f), 1.0f, 2.0f);
        Cylinder cylinder2 = new Cylinder(new Vector3f(5.0f, 5.0f, 5.0f), 1.0f, 2.0f);
        assertFalse(CDSATGJK.isCollide(cylinder1, cylinder2));
    }

    @Test
    public void testCollidingPlanes() {
        Plane plane1 = new Plane(new Vector3f(1.0f, 0.0f, 0.0f), 0.0f);
        Plane plane2 = new Plane(new Vector3f(2.0f, 0.0f, 0.0f), 0.0f);
        assertTrue(CDSATGJK.isCollide(plane1, plane2));
    }

    @Test
    public void testNonCollidingPlanes() {
        Plane plane1 = new Plane(new Vector3f(1.0f, 0.0f, 0.0f), 0.0f);
        Plane plane2 = new Plane(new Vector3f(0.0f, 1.0f, 0.0f), 0.0f);
        assertFalse(CDSATGJK.isCollide(plane1, plane2));
    }

    @Test
    void testIsSphereCollidingWithSphere() {
        Sphere sphere1 = new Sphere(new Vector3f(0, 0, 0), 1.0f);
        Sphere sphere2 = new Sphere(new Vector3f(1, 1, 1), 1.0f);

        boolean result = CDSATGJK.isCollide(sphere1, sphere2);

        assertTrue(result, "Spheres should be colliding");
    }

    @Test
    void testIsSphereNoCollidingWithSphere() {
        Sphere sphere1 = new Sphere(new Vector3f(0, 0, 0), 1.0f);
        Sphere sphere2 = new Sphere(new Vector3f(3, 3, 3), 1.0f);

        boolean result = CDSATGJK.isCollide(sphere1, sphere2);

        assertFalse(result, "Spheres should not be colliding");
    }


}