package collisionDetection.narrowPhase;

import collisionDetection.primitive.*;
import math.Vector3f;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class CollisionDetectionTest {

    @Test
    public void testAABBCollisionLine() {
        // Create an AABB with minimum corner (-1, -1, -1) and maximum corner (1, 1, 1)
        AABB aabb = new AABB(new Vector3f(-1, -1, -1), new Vector3f(1, 1, 1));

        // Test cases where the line intersects the AABB
        Line line1 = new Line(new Vector3f(0, 0, -2), new Vector3f(0, 0, 2));
        assertTrue(CollisionDetection.isCollide(line1, aabb));

        Line line2 = new Line(new Vector3f(-2, -2, -2), new Vector3f(-1, -1, -1));
        assertTrue(CollisionDetection.isCollide(line2, aabb));

        Line line3 = new Line(new Vector3f(0, 0, 0), new Vector3f(0, 0, 1));
        assertTrue(CollisionDetection.isCollide(line3, aabb));

        // Test cases where the line does not intersect the AABB
        Line line4 = new Line(new Vector3f(2, 2, 2), new Vector3f(3, 3, 3));
        assertFalse(CollisionDetection.isCollide(line4, aabb));

        Line line5 = new Line(new Vector3f(2, 2, 2), new Vector3f(2, 2, 3));
        assertFalse(CollisionDetection.isCollide(line5, aabb));

        Line line6 = new Line(new Vector3f(2, 0, 0), new Vector3f(3, 0, 0));
        assertFalse(CollisionDetection.isCollide(line6, aabb));
    }

    @Test
    public void testCapsuleCollisionLine() {
        // Create a capsule with start point (-1, 0, 0), end point (1, 0, 0), and radius 0.5
        Capsule capsule = new Capsule(new Vector3f(-1, 0, 0), new Vector3f(1, 0, 0), 0.5f);

        // Test cases where the line collides with the capsule
        Line line1 = new Line(new Vector3f(0, 0, -1), new Vector3f(0, 0, 1));
        assertTrue(CollisionDetection.isCollide(line1, capsule));

        Line line2 = new Line(new Vector3f(0, 0, 0), new Vector3f(0, 0, 1));
        assertTrue(CollisionDetection.isCollide(line2, capsule));

        Line line3 = new Line(new Vector3f(-2, 0, 0), new Vector3f(-1, 0, 0));
        assertTrue(CollisionDetection.isCollide(line3, capsule));

        Line line4 = new Line(new Vector3f(0, 0, 2), new Vector3f(0, 0, 3));
        assertFalse(CollisionDetection.isCollide(line4, capsule));

        Line line5 = new Line(new Vector3f(2, 0, 0), new Vector3f(3, 0, 0));
        assertFalse(CollisionDetection.isCollide(line5, capsule));

        Line line6 = new Line(new Vector3f(0, 0, -2), new Vector3f(0, 0, -1));
        assertFalse(CollisionDetection.isCollide(line6, capsule));
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
        assertTrue(CollisionDetection.isCollide(line1, cube));

        Line line2 = new Line(new Vector3f(0, 0, 0), new Vector3f(0, 0, 1));
        assertTrue(CollisionDetection.isCollide(line2, cube));

        Line line3 = new Line(new Vector3f(-2, -2, -2), new Vector3f(-1, -1, -1));
        assertTrue(CollisionDetection.isCollide(line3, cube));

        // Test cases where the line does not collide with the cube
        Line line4 = new Line(new Vector3f(2, 2, 2), new Vector3f(4, 4, 4));
        assertFalse(CollisionDetection.isCollide(line4, cube));

        Line line5 = new Line(new Vector3f(2, 2, 2), new Vector3f(3, 3, 3));
        assertFalse(CollisionDetection.isCollide(line5, cube));

        Line line6 = new Line(new Vector3f(2, 2, 2), new Vector3f(2, 2, 3));
        assertFalse(CollisionDetection.isCollide(line6, cube));
    }

    @Test
    public void testSphereCollisionLine() {
        // Create a sphere with center (0, 0, 0) and radius 1.
        Sphere sphere = new Sphere(new Vector3f(0, 0, 0), 1);

        // Test a line that collides with the sphere.
        Line line1 = new Line(new Vector3f(2, 0, 0), new Vector3f(-2, 0, 0));
        assertTrue(CollisionDetection.isCollide(line1, sphere));

        // Test a line that does not collide with the sphere.
        Line line2 = new Line(new Vector3f(2, 2, 0), new Vector3f(3, 3, 0));
        assertFalse(CollisionDetection.isCollide(line2, sphere));

        // Test a line that starts inside the sphere.
        Line line3 = new Line(new Vector3f(0, 0, 0), new Vector3f(1, 1, 1));
        assertTrue(CollisionDetection.isCollide(line3, sphere));

        // Test a line that ends inside the sphere.
        Line line4 = new Line(new Vector3f(2, 2, 2), new Vector3f(0, 0, 0));
        assertTrue(CollisionDetection.isCollide(line4, sphere));

        // Test a line that is completely outside the sphere.
        Line line5 = new Line(new Vector3f(3, 3, 3), new Vector3f(4, 4, 4));
        assertFalse(CollisionDetection.isCollide(line5, sphere));
    }

    @Test
    public void testCylinderCollisionLine() {
        // Create a cylinder with center (0, 0, 0), radius 1, and height 2.
        Cylinder cylinder = new Cylinder(new Vector3f(0, 0, 0), 1, 2);

        // Test a line that collides with the cylinder.
        Line line1 = new Line(new Vector3f(0, 1, 0), new Vector3f(0, -1, 0));
        assertTrue(CollisionDetection.isCollide(line1, cylinder));

        // Test a line that does not collide with the cylinder.
        Line line2 = new Line(new Vector3f(2, 2, 2), new Vector3f(3, 3, 3));
        assertFalse(CollisionDetection.isCollide(line2, cylinder));

        // Test a line that starts inside the cylinder.
        Line line3 = new Line(new Vector3f(0, 0, 0), new Vector3f(0, 1, 0));
        assertTrue(CollisionDetection.isCollide(line3, cylinder));

        // Test a line that ends inside the cylinder.
        Line line4 = new Line(new Vector3f(2, 2, 0), new Vector3f(0, 0, 0));
        assertTrue(CollisionDetection.isCollide(line4, cylinder));

        // Test a line that is completely outside the cylinder.
        Line line5 = new Line(new Vector3f(2, 2, 2), new Vector3f(4, 4, 4));
        assertFalse(CollisionDetection.isCollide(line5, cylinder));
    }

    @Test
    public void testPlaneCollisionLine() {
        // Create a plane with a normal vector (0, 1, 0) and a distance of 2 units from the origin.
        Plane plane = new Plane(new Vector3f(0, 1, 0), 2);

        // Test a line that collides with the plane.
        Line line1 = new Line(new Vector3f(0, 0, 0), new Vector3f(0, 3, 0));
        assertTrue(CollisionDetection.isCollide(line1, plane));

        // Test a line that does collide with the plane.
        Line line2 = new Line(new Vector3f(1, 1, 1), new Vector3f(2, 2, 2));
        assertTrue(CollisionDetection.isCollide(line2, plane));

        // Test a line that is parallel to the plane (no collision).
        Line line3 = new Line(new Vector3f(0, 0, 0), new Vector3f(1, 0, 1));
        assertFalse(CollisionDetection.isCollide(line3, plane));

        // Test a line that intersects the plane at its endpoint.
        Line line4 = new Line(new Vector3f(0, 2, 0), new Vector3f(0, 3, 0));
        assertTrue(CollisionDetection.isCollide(line4, plane));

        // Create a plane with a known normal and distance
        Plane plane2 = new Plane(new Vector3f(0, 0, 1), 0);

        // Test a line that intersects with the plane
        Line intersectingLine = new Line(new Vector3f(0, 0, -1), new Vector3f(0, 0, 1));
        assertTrue(CollisionDetection.isCollide(intersectingLine, plane2));

        // Test a line that is parallel to the plane and does not intersect
        Line parallelLine = new Line(new Vector3f(1, 1, 1), new Vector3f(2, 2, 2));
        assertFalse(CollisionDetection.isCollide(parallelLine, plane2));
    }

    @Test
    public void testOBBCollisionLine() {
        // Create an OBB with a known center and half extents
        OBB obb = new OBB(new Vector3f(1, 1, 1), new Vector3f(1, 1, 1));

        // Test a line that intersects with the OBB
        Line intersectingLine = new Line(new Vector3f(0, 0, 0), new Vector3f(2, 2, 2));
        assertTrue(CollisionDetection.isCollide(intersectingLine, obb));

        // Test a line that does not intersect with the OBB
        Line nonIntersectingLine = new Line(new Vector3f(3, 3, 0), new Vector3f(5, 5, 0));
        assertFalse(CollisionDetection.isCollide(nonIntersectingLine, obb));

        // Create an OBB
        OBB obb2 = new OBB(new Vector3f(0.0f, 0.0f, 0.0f), new Vector3f(1.0f, 1.0f, 1.0f));

        // Create a line that intersects with the OBB
        Line line1 = new Line(new Vector3f(-2.0f, 0.0f, 0.0f), new Vector3f(2.0f, 0.0f, 0.0f));

        // Create a line that doesn't intersect with the OBB
        Line line2 = new Line(new Vector3f(-2.0f, 2.0f, 0.0f), new Vector3f(-1.0f, 2.0f, 0.0f));

        assertTrue(CollisionDetection.isCollide(line1, obb2)); // Expect collision
        assertFalse(CollisionDetection.isCollide(line2, obb2)); // Expect no collision
    }

    @Test
    public void testTriangleCollisionLine() {
        // Create a simple triangle and line segment for testing
        Vector3f vertex1 = new Vector3f(0, 0, 0);
        Vector3f vertex2 = new Vector3f(2, 0, 0);
        Vector3f vertex3 = new Vector3f(1, 2, 0);

        Triangle triangle = new Triangle(vertex1, vertex2, vertex3);
        Line line = new Line(new Vector3f(0.5f, 0.5f, 1), new Vector3f(1.5f, 1.5f, 1));

        // Test if the line and triangle intersect
        assertTrue(CollisionDetection.isCollide(line, triangle));

        // Test non-intersecting case
        Line nonIntersectingLine = new Line(new Vector3f(3, 3, 3), new Vector3f(4, 4, 4));
        assertFalse(CollisionDetection.isCollide(nonIntersectingLine, triangle));

        Line line2 = new Line(new Vector3f(0.5f, 0.5f, 0), new Vector3f(1.5f, 1.5f, 0));

        assertTrue(CollisionDetection.isCollide(line2, triangle));

        // Create a triangle entirely inside a line segment
        vertex1 = new Vector3f(0, 0, 0);
        vertex2 = new Vector3f(4, 0, 0);
        vertex3 = new Vector3f(2, 2, 0);

        Triangle triangle2 = new Triangle(vertex1, vertex2, vertex3);
        Line line3 = new Line(new Vector3f(1, 0.5f, 1), new Vector3f(3, 0.5f, 1));

        assertTrue(CollisionDetection.isCollide(line3, triangle2));
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
        assertTrue(CollisionDetection.isCollide(line, terrainShape));

        // Create a Line object representing a line segment that does not collide with the terrain
        Line line2 = new Line(new Vector3f(3, 3, 3), new Vector3f(4, 4, 4));

        // Assert that the result is false since the line should not collide with the terrain
        assertFalse(CollisionDetection.isCollide(line2, terrainShape));
    }

    @Test
    void testAABBCollidingWithSphere() {
        Vector3f min = new Vector3f(-1, -1, -1);
        Vector3f max = new Vector3f(1, 1, 1);
        AABB aabb = new AABB(min, max);

        Sphere sphere = new Sphere(new Vector3f(0, 1.5f, 0), 1.0f);

        boolean result = CollisionDetection.isCollide(sphere, aabb);

        assertTrue(result, "AABB Sphere should be colliding");
    }

    @Test
    void testAABBNotCollidingWithSphere() {
        Vector3f min = new Vector3f(-1, -1, -1);
        Vector3f max = new Vector3f(1, 1, 1);
        AABB aabb = new AABB(min, max);

        Sphere sphere = new Sphere(new Vector3f(2, 2, 2), 0.5f);
        boolean result = CollisionDetection.isCollide(sphere, aabb);

        assertFalse(result, "AABB Sphere should not be colliding");
    }


    @Test
    public void testSphereCollidingWithPlane() {
        Sphere sphere = new Sphere(new Vector3f(0.0f, 4.0f, 0.0f), 2.0f);
        Plane plane = new Plane(new Vector3f(0.0f, 1.0f, 0.0f), 3.0f);

        boolean result = CollisionDetection.isCollide(sphere, plane);

        assertTrue(result,"Sphere Plane should be colliding");
    }

    @Test
    public void testSphereNotCollidingWithPlane() {
        Sphere sphere = new Sphere(new Vector3f(0.0f, 5.0f, 0.0f), 2.0f);
        Plane plane = new Plane(new Vector3f(1.0f, 0.0f, 0.0f), 10.0f);

        boolean result = CollisionDetection.isCollide(sphere, plane);

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
        boolean result = CollisionDetection.isCollide(sphere, triangle);
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

        boolean result = CollisionDetection.isCollide(sphere, triangle);

        assertFalse(result, "Sphere should not be colliding with the triangle");
    }

    @Test
    void testOBBCollidingWithSphere() {
        Vector3f center1 = new Vector3f(0.0f, 0.0f, 0.0f);
        Vector3f halfExtents1 = new Vector3f(1.0f, 1.0f, 1.0f);
        OBB obb = new OBB(center1, halfExtents1);

        Sphere sphere = new Sphere(new Vector3f(0, 1.5f, 0), 1.0f);

        boolean result = CollisionDetection.isCollide(sphere, obb);

        assertTrue(result, "OBB Sphere should be colliding");
    }

    @Test
    void testOBBNotCollidingWithSphere() {
        Vector3f center1 = new Vector3f(8.0f, 8.0f, 8.0f);
        Vector3f halfExtents1 = new Vector3f(1.0f, 1.0f, 1.0f);
        OBB obb = new OBB(center1, halfExtents1);


        Sphere sphere = new Sphere(new Vector3f(2, 2, 2), 0.5f);
        boolean result = CollisionDetection.isCollide(sphere, obb);

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

        boolean collision = CollisionDetection.isCollide(aabb, obb);

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

        boolean collision = CollisionDetection.isCollide(aabb, obb);

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

        boolean collision = CollisionDetection.isCollide(sphere, capsule);

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

        boolean collision = CollisionDetection.isCollide(sphere, capsule);

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

        boolean collision = CollisionDetection.isCollide(sphere, cylinder);

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

        boolean collision = CollisionDetection.isCollide(sphere, cylinder);

        assertFalse(collision,"Sphere Cylinder should not be colliding");
    }

    @Test
    void testPlaneCollidingWithAABB() {
        Plane plane = new Plane(new Vector3f(0, 1, 0), 0); // Plane with normal (0, 1, 0) and distance 0
        AABB aabb = new AABB(new Vector3f(-1, -1, -1), new Vector3f(1, 1, 1)); // AABB centered at (0, 0, 0)

        boolean result = CollisionDetection.isCollide(plane, aabb);
        assertTrue(result, "AABB and plane should be colliding");
    }

    @Test
    void testPlaneNotCollidingWithAABB() {
        Plane plane = new Plane(new Vector3f(0, 1, 0), 0); // Plane with normal (0, 1, 0) and distance 0
        AABB aabb = new AABB(new Vector3f(-2, -2, -2), new Vector3f(-1, -1, -1)); // AABB above the plane

        boolean result = CollisionDetection.isCollide(plane, aabb);
        assertFalse(result, "AABB and plane should not be colliding");
    }

    @Test
    void testCapsuleCollidingWithAABB() {
        Capsule capsule = new Capsule(new Vector3f(0, 0, 0), new Vector3f(0, 2, 0), 1.0f);
        AABB aabb = new AABB(new Vector3f(-1, -1, -1), new Vector3f(1, 1, 1));

        boolean result =CollisionDetection.isCollide(capsule, aabb);
        assertTrue(result, "Capsule and AABB should be colliding");
    }

    @Test
    void testCapsuleNotCollidingWithAABB() {
        Capsule capsule = new Capsule(new Vector3f(1, 1, 3), new Vector3f(1, 2, 3), 1.0f);
        AABB aabb = new AABB(new Vector3f(-1, -1, -1), new Vector3f(1, 1, 1));

        boolean result = CollisionDetection.isCollide(capsule, aabb);
        assertFalse(result, "Capsule and AABB should not be colliding");
    }

    @Test
    void testCylinderCollidingWithAABB() {
        Cylinder cylinder = new Cylinder(new Vector3f(0, 1, 0), 1.0f, 4.0f);
        AABB aabb = new AABB(new Vector3f(-1, 0, -1), new Vector3f(1, 3, 1));

        boolean result = CollisionDetection.isCollide(cylinder, aabb);
        assertTrue(result, "Cylinder and AABB should be colliding");
    }

    @Test
    void testCylinderNotCollidingWithAABB() {
        Cylinder cylinder = new Cylinder(new Vector3f(0, 5, 0), 1.0f, 2.0f);
        AABB aabb = new AABB(new Vector3f(-1, 0, -1), new Vector3f(1, 3, 1));

        boolean result = CollisionDetection.isCollide(cylinder, aabb);
        assertFalse(result, "Cylinder and AABB should not be colliding");
    }

    @Test
    void testCapsuleCollidingWithCylinder() {
        Capsule capsule = new Capsule(new Vector3f(0, 1, 0), new Vector3f(0, 3, 0), 0.5f);
        Cylinder cylinder = new Cylinder(new Vector3f(0, 2, 0), 1.0f, 4.0f);

        boolean result = CollisionDetection.isCollide(capsule, cylinder);
        assertTrue(result, "Capsule and Cylinder should be colliding");
    }

    @Test
    void testCapsuleNotCollidingWithCylinder() {
        Capsule capsule = new Capsule(new Vector3f(0, 10, 0), new Vector3f(0, 12, 0), 1f);
        Cylinder cylinder = new Cylinder(new Vector3f(0, 2, 0), 1.0f, 4.0f);

        boolean result = CollisionDetection.isCollide(capsule, cylinder);
        assertFalse(result, "Capsule and Cylinder should not be colliding");
    }

    @Test
    void testCylinderCollidingWithPlane() {
        Cylinder cylinder = new Cylinder(new Vector3f(0, 1, 0), 1.0f, 2.0f);
        Plane plane = new Plane(new Vector3f(0, 1, 0), 0); // Plane with normal (0, 1, 0) and distance 0

        boolean result = CollisionDetection.isCollide(cylinder, plane);
        assertTrue(result, "Cylinder and Plane should be colliding");
    }

    @Test
    void testCylinderNotCollidingWithPlane() {
        Cylinder cylinder = new Cylinder(new Vector3f(0, 5, 0), 1.0f, 2.0f);
        Plane plane = new Plane(new Vector3f(0, 1, 0), 0); // Plane with normal (0, 1, 0) and distance 0

        boolean result = CollisionDetection.isCollide(cylinder, plane);
        assertFalse(result, "Cylinder and Plane should not be colliding");
    }

    @Test
    void testCapsuleCollidingWithPlane() {
        Capsule capsule = new Capsule(new Vector3f(0, 1, 0), new Vector3f(0, 3, 0), 0.5f);
        Plane plane = new Plane(new Vector3f(0, 1, 0), 0); // Plane with normal (0, 1, 0) and distance 0

        boolean result = CollisionDetection.isCollide(capsule, plane);
        assertTrue(result, "Capsule and Plane should be colliding");
    }

    @Test
    void testCapsuleNotCollidingWithPlane() {
        Capsule capsule = new Capsule(new Vector3f(0, 5, 0), new Vector3f(0, 7, 0), 0.5f);
        Plane plane = new Plane(new Vector3f(0, 1, 0), 0); // Plane with normal (0, 1, 0) and distance 0

        boolean result = CollisionDetection.isCollide(capsule, plane);
        assertFalse(result, "Capsule and Plane should not be colliding");
    }

    @Test
    void testCylinderCollidingWithTriangle() {
        Cylinder cylinder = new Cylinder(new Vector3f(0, 0, 0), 1.0f, 2.0f);

        Triangle triangle = new Triangle(new Vector3f(0, 0, 0), new Vector3f(1, 0, 0), new Vector3f(0, 1, 0));

        boolean collision = CollisionDetection.isCollide(cylinder, triangle);
        assertTrue(collision, "Expected collision between cylinder and triangle");
    }

    @Test
    void testCylinderNotCollidingWithTriangle() {
        Cylinder cylinder = new Cylinder(new Vector3f(5, 5, 5), 1.0f, 2.0f);

        Triangle triangle = new Triangle(new Vector3f(-3, 0, 0), new Vector3f(-2, 0, 0), new Vector3f(-2, -1, 0));

        boolean collision = CollisionDetection.isCollide(cylinder, triangle);
        assertFalse(collision, "Expected no collision between cylinder and triangle");
    }

    @Test
    void testCylinderCollidingWithOBB() {
        Cylinder cylinder = new Cylinder(new Vector3f(0, 0, 0), 1.0f, 2.0f);
        OBB obb = new OBB(new Vector3f(1, 0, 0), new Vector3f(1, 1, 1));

        boolean result = CollisionDetection.isCollide(cylinder, obb);
        assertTrue(result, "Collision should be detected");
    }

    @Test
    void testCylinderNotCollidingWithOBB() {
        Cylinder cylinder = new Cylinder(new Vector3f(0, 0, 0), 1.0f, 2.0f);
        OBB obb = new OBB(new Vector3f(4, 4, 4), new Vector3f(1, 1, 1));

        boolean result = CollisionDetection.isCollide(cylinder, obb);
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

        assertTrue(CollisionDetection.isCollide(triangle, aabb));
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

        assertFalse(CollisionDetection.isCollide(triangle, aabb));
    }

    @Test
    void testPlaneCollidingWithTriangle() {
        Vector3f normal = new Vector3f(0, 1, 0);
        Plane plane = new Plane(normal, 0);

        Vector3f vertex1 = new Vector3f(0, 0, 0);
        Vector3f vertex2 = new Vector3f(1, 0, 0);
        Vector3f vertex3 = new Vector3f(0.5f, 1, 0);
        Triangle triangle = new Triangle(vertex1, vertex2, vertex3);

        assertTrue(CollisionDetection.isCollide(triangle, plane));
    }

    @Test
    void testPlaneNotCollidingWithTriangle() {
        Vector3f normal = new Vector3f(0, 1, 0);
        Plane plane = new Plane(normal, 0);

        Vector3f vertex1 = new Vector3f(0, 0, 0);
        Vector3f vertex2 = new Vector3f(1, 0, 0);
        Vector3f vertex3 = new Vector3f(2, 0, 0);
        Triangle triangle = new Triangle(vertex1, vertex2, vertex3);

        assertFalse(CollisionDetection.isCollide(triangle, plane));
    }


    @Test
    public void testCapsuleCollidingWithOBB() {
        // Create a colliding capsule and OBB
        Capsule capsule = new Capsule(new Vector3f(0, 0, 0), new Vector3f(0, 1, 0), 0.5f);
        OBB obb = new OBB(new Vector3f(0, 0.5f, 0), new Vector3f(0.5f, 0.5f, 0.5f));

        assertTrue(CollisionDetection.isCollide(capsule, obb));
    }

    @Test
    public void testCapsuleNotCollidingWithOBB() {
        // Create a non-colliding capsule and OBB
        Capsule capsule = new Capsule(new Vector3f(3, 3, 0), new Vector3f(5, 5, 0), 0.5f);
        OBB obb = new OBB(new Vector3f(0, 0, 0), new Vector3f(0.5f, 0.5f, 0.5f));

        assertFalse(CollisionDetection.isCollide(capsule, obb));
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

        assertTrue(CollisionDetection.isCollide(capsule, triangle));
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

        assertFalse(CollisionDetection.isCollide(capsule, triangle));
    }

    @Test
    public void testIsCapsuleColliding() {
        Capsule capsule3 = new Capsule(new Vector3f(0, 0, 0), new Vector3f(0, 3, 0), 1.0f);
        Capsule capsule4 = new Capsule(new Vector3f(0, 2, 0), new Vector3f(0, 5, 0), 1.0f);

        boolean result = CollisionDetection.isCollide(capsule3, capsule4);
        // Assert that the capsules are not colliding
        assertTrue(result, "Capsules should not be colliding");
    }

    @Test
    public void testIsCapsuleNotColliding() {
        Capsule capsule1 = new Capsule(new Vector3f(0, 0, 0), new Vector3f(0, 3, 0), 1.0f);
        Capsule capsule2 = new Capsule(new Vector3f(2, 2, 0), new Vector3f(2, 5, 0), 1.0f);

        boolean result = CollisionDetection.isCollide(capsule1, capsule2);

        assertFalse(result, "Capsules not be should be colliding");
    }

    @Test
    void testNonCollidingPolyhedra() {
        ConvexPolyhedron polyhedron1 = createNonCollidingPolyhedron1();
        ConvexPolyhedron polyhedron2 = createNonCollidingPolyhedron2();

        assertFalse(CollisionDetection.isCollide(polyhedron1, polyhedron2));
    }

    @Test
    void testCollidingPolyhedra() {
        ConvexPolyhedron polyhedron1 = createCollidingPolyhedron1();
        ConvexPolyhedron polyhedron2 = createCollidingPolyhedron2();

        assertTrue(CollisionDetection.isCollide(polyhedron1, polyhedron2));
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
    public void testCollidingCylinders() {
        Cylinder cylinder1 = new Cylinder(new Vector3f(1.0f, 1.0f, 1.0f), 1.0f, 2.0f);
        Cylinder cylinder2 = new Cylinder(new Vector3f(2.0f, 2.0f, 2.0f), 1.0f, 2.0f);
        assertTrue(CollisionDetection.isCollide(cylinder1, cylinder2));
    }

    @Test
    public void testNonCollidingCylinders() {
        Cylinder cylinder1 = new Cylinder(new Vector3f(1.0f, 1.0f, 1.0f), 1.0f, 2.0f);
        Cylinder cylinder2 = new Cylinder(new Vector3f(5.0f, 5.0f, 5.0f), 1.0f, 2.0f);
        assertFalse(CollisionDetection.isCollide(cylinder1, cylinder2));
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

        assertTrue(CollisionDetection.isCollide(obb1, obb2),"OBBs should be colliding");
    }

    @Test
    public void testNonCollidingOBBs() {
        Vector3f center1 = new Vector3f(0.0f, 0.0f, 0.0f);
        Vector3f halfExtents1 = new Vector3f(1.0f, 1.0f, 1.0f);
        OBB obb1 = new OBB(center1, halfExtents1);

        Vector3f center2 = new Vector3f(3.0f, 3.0f, 3.0f);
        Vector3f halfExtents2 = new Vector3f(1.0f, 1.0f, 1.0f);
        OBB obb2 = new OBB(center2, halfExtents2);

        boolean collision = CollisionDetection.isCollide(obb1, obb2);

        assertFalse(collision);
    }

    @Test
    public void testNonParallelEdges() {
        // Create two non-parallel OBBs
        OBB obb1 = new OBB(new Vector3f(0, 0, 0),new Vector3f(1, 1, 1));
        OBB obb2 = new OBB(new Vector3f(4, 4, 4),new Vector3f(1, 1, 1));

        assertFalse(CollisionDetection.isCollide(obb1, obb2), "Non-parallel OBBs should not be colliding");
    }

    @Test
    public void testLineCollision() {
        // Test cases where the lines intersect
        Line line1 = new Line(new Vector3f(0, 0, 0), new Vector3f(1, 1, 0));
        Line line2 = new Line(new Vector3f(0, 1, 0), new Vector3f(1, 0, 0));
        assertTrue(CollisionDetection.isCollide(line1, line2));

        Line line3 = new Line(new Vector3f(0, 0, 0), new Vector3f(1, 1, 0));
        Line line4 = new Line(new Vector3f(0.5f, 0.5f, 0), new Vector3f(1, 0, 0));
        assertTrue(CollisionDetection.isCollide(line3, line4));

        // Test cases where the lines do not intersect
        Line line5 = new Line(new Vector3f(0, 0, 0), new Vector3f(1, 1, 0));
        Line line6 = new Line(new Vector3f(2, 2, 0), new Vector3f(3, 3, 0));
        assertFalse(CollisionDetection.isCollide(line5, line6));

        Line line7 = new Line(new Vector3f(0, 0, 0), new Vector3f(1, 1, 0));
        Line line8 = new Line(new Vector3f(0, 2, 0), new Vector3f(1, 3, 0));
        assertFalse(CollisionDetection.isCollide(line7, line8));

        // Test cases where the lines are parallel but not collinear
        Line line9 = new Line(new Vector3f(0, 0, 0), new Vector3f(1, 1, 0));
        Line line10 = new Line(new Vector3f(0, 0, 1), new Vector3f(1, 1, 1));
        assertFalse(CollisionDetection.isCollide(line9, line10));

        Line line11 = new Line(new Vector3f(2, 2, 2), new Vector3f(4, 4, 4));
        Line line12 = new Line(new Vector3f(1, -1, 1), new Vector3f(1, 1, 1));
        assertFalse(CollisionDetection.isCollide(line11, line12));
    }

    @Test
    public void testCollidingPlanes() {
        Plane plane1 = new Plane(new Vector3f(1.0f, 0.0f, 0.0f), 0.0f);
        Plane plane2 = new Plane(new Vector3f(2.0f, 0.0f, 0.0f), 0.0f);
        assertTrue(CollisionDetection.isCollide(plane1, plane2));
    }

    @Test
    public void testNonCollidingPlanes() {
        Plane plane1 = new Plane(new Vector3f(1.0f, 0.0f, 0.0f), 0.0f);
        Plane plane2 = new Plane(new Vector3f(0.0f, 1.0f, 0.0f), 0.0f);
        assertFalse(CollisionDetection.isCollide(plane1, plane2));
    }

    @Test
    void testIsSphereCollidingWithSphere() {
        Sphere sphere1 = new Sphere(new Vector3f(0, 0, 0), 1.0f);
        Sphere sphere2 = new Sphere(new Vector3f(1, 1, 1), 1.0f);

        boolean result = CollisionDetection.isCollide(sphere1, sphere2);

        assertTrue(result, "Spheres should be colliding");
    }

    @Test
    void testIsSphereNoCollidingWithSphere() {
        Sphere sphere1 = new Sphere(new Vector3f(0, 0, 0), 1.0f);
        Sphere sphere2 = new Sphere(new Vector3f(3, 3, 3), 1.0f);

        boolean result = CollisionDetection.isCollide(sphere1, sphere2);

        assertFalse(result, "Spheres should not be colliding");
    }

    @Test
    public void testIntersects() {
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
        assertTrue(CollisionDetection.isCollide(triangle1, triangle2),"Triangles should be colliding");
    }

    @Test
    public void testDoesNotIntersect() {
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

        assertFalse(CollisionDetection.isCollide(triangle1, triangle2),"Triangles should not be colliding");
    }

    @Test
    public void  parallelEdges() {
        Triangle triangle1 = new Triangle(
                new Vector3f(-2,-1,0),
                new Vector3f(-3, 0, 0),
                new Vector3f(-1, 0,0)
        );

        Triangle triangle2 = new Triangle(
                new Vector3f(2, 1, 0),
                new Vector3f(3, 0, 0),
                new Vector3f(1, 0, 0)
        );

        assertFalse(CollisionDetection.isCollide(triangle1, triangle2),"Triangles not be colliding");
    }

    @Test
    void testIsAABBCollidingWithAABB() {
        Vector3f min1 = new Vector3f(0, 0, 0);
        Vector3f max1 = new Vector3f(2, 2, 2);
        AABB aabb1 = new AABB(min1, max1);

        Vector3f min2 = new Vector3f(1, 1, 1);
        Vector3f max2 = new Vector3f(3, 3, 3);
        AABB aabb2 = new AABB(min2, max2);

        boolean result = CollisionDetection.isCollide(aabb1, aabb2);

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

        boolean result = CollisionDetection.isCollide(aabb1, aabb2);

        assertFalse(result, "AABBs should not be colliding");
    }

}