package collisionDetection.narrowPhase.cd;

import collisionDetection.CDEngine;
import collisionDetection.broadPhase.SAP;
import collisionDetection.primitive.*;
import collisionDetection.primitive.terrain.TerrainShape;
import math.Vector3f;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class CDSATGJKTest {

    static CDSatGjk cdSatGjk=new CDSatGjk();

    @Test
    public void testPlaneCollisionLine() {
        // Create a plane with a normal vector (0, 1, 0) and a distance of 2 units from the origin.
        Plane plane = new Plane(new Vector3f(0, 1, 0), 2);

        // Test a line that collides with the plane.
        Line line1 = new Line(new Vector3f(0, 0, 0), new Vector3f(0, 3, 0));
        assertTrue(cdSatGjk.isCollide(plane, line1));

        // Test a line that does collide with the plane.
        Line line2 = new Line(new Vector3f(1, 1, 1), new Vector3f(2, 2, 2));
        assertTrue(cdSatGjk.isCollide(plane, line2));

        // Test a line that is parallel to the plane (no collision).
        Line line3 = new Line(new Vector3f(0, 0, 0), new Vector3f(1, 0, 1));
        assertFalse(cdSatGjk.isCollide(plane, line3));

        // Test a line that intersects the plane at its endpoint.
        Line line4 = new Line(new Vector3f(0, 2, 0), new Vector3f(0, 3, 0));
        assertTrue(cdSatGjk.isCollide(plane, line4));

        // Create a plane with a known normal and distance
        Plane plane2 = new Plane(new Vector3f(0, 0, 1), 0);

        // Test a line that intersects with the plane
        Line intersectingLine = new Line(new Vector3f(0, 0, -1), new Vector3f(0, 0, 1));
        assertTrue(cdSatGjk.isCollide(plane2, intersectingLine));

        // Test a line that is parallel to the plane and does not intersect
        Line parallelLine = new Line(new Vector3f(1, 1, 1), new Vector3f(2, 2, 2));
        assertFalse(cdSatGjk.isCollide(plane2, parallelLine));
    }

    @Test
    public void testTerrainCollisionLine() {
        CDEngine.init(new SAP());
        // Create a TerrainShape with height data, borders, width, and length for testing collision
        float[][] heightData = new float[5][5];

        // Define a simple AABB for terrain borders
        AABB borders = new AABB(new Vector3f(-1, -1, -1), new Vector3f(5, 5, 5));

        // Initialize the TerrainShape object
        TerrainShape terrainShape = new TerrainShape(heightData, borders, new Vector3f(1, 1, 1));

        // Create a Line object representing a line segment that collides with the terrain
        Line line = new Line(new Vector3f(0, 5, 0), new Vector3f(0, -1, 0));

        // Assert that the result is true since the line should collide with the terrain
        assertTrue(cdSatGjk.isCollide(line, terrainShape));

        // Create a Line object representing a line segment that does not collide with the terrain
        Line line2 = new Line(new Vector3f(3, 3, 3), new Vector3f(4, 4, 4));

        // Assert that the result is false since the line should not collide with the terrain
        assertFalse(cdSatGjk.isCollide(line2, terrainShape));
    }


    @Test
    public void testSphereCollidingWithPlane() {
        Sphere sphere = new Sphere(new Vector3f(0.0f, 4.0f, 0.0f), 2.0f);
        Plane plane = new Plane(new Vector3f(0.0f, 1.0f, 0.0f), 3.0f);

        boolean result = cdSatGjk.isCollide(plane, sphere);

        assertTrue(result, "Sphere Plane should be colliding");
    }

    @Test
    public void testSphereNotCollidingWithPlane() {
        Sphere sphere = new Sphere(new Vector3f(0.0f, 5.0f, 0.0f), 2.0f);
        Plane plane = new Plane(new Vector3f(1.0f, 0.0f, 0.0f), 10.0f);

        boolean result = cdSatGjk.isCollide(plane, sphere);

        assertFalse(result, "Sphere Plane should not be colliding");
    }


    @Test
    void testOBBCollidingWithSphere() {
        Vector3f center1 = new Vector3f(0.0f, 0.0f, 0.0f);
        Vector3f halfExtents1 = new Vector3f(1.0f, 1.0f, 1.0f);
        OBB obb = new OBB(center1, halfExtents1);

        Sphere sphere = new Sphere(new Vector3f(0, 0, 0), 1.0f);

        boolean result = cdSatGjk.isCollide(sphere, obb);

        assertTrue(result, "OBB Sphere should be colliding");
    }

    @Test
    void testOBBNotCollidingWithSphere() {
        Vector3f center1 = new Vector3f(8.0f, 8.0f, 8.0f);
        Vector3f halfExtents1 = new Vector3f(1.0f, 1.0f, 1.0f);
        OBB obb = new OBB(center1, halfExtents1);


        Sphere sphere = new Sphere(new Vector3f(2, 2, 2), 0.5f);
        boolean result = cdSatGjk.isCollide(sphere, obb);

        assertFalse(result, "OBB Sphere should not be colliding");
    }


    @Test
    void testPlaneCollidingWithAABB() {
        Plane plane = new Plane(new Vector3f(0, 1, 0), 0); // Plane with normal (0, 1, 0) and distance 0
        AABB aabb = new AABB(new Vector3f(-1, -1, -1), new Vector3f(1, 1, 1)); // AABB centered at (0, 0, 0)

        boolean result = cdSatGjk.isCollide(plane, aabb);
        assertTrue(result, "AABB and plane should be colliding");
    }

    @Test
    void testPlaneNotCollidingWithAABB() {
        Plane plane = new Plane(new Vector3f(0, 1, 0), 0); // Plane with normal (0, 1, 0) and distance 0
        AABB aabb = new AABB(new Vector3f(-2, -2, -2), new Vector3f(-1, -1, -1)); // AABB above the plane

        boolean result = cdSatGjk.isCollide(plane, aabb);
        assertFalse(result, "AABB and plane should not be colliding");
    }

    @Test
    void testCapsuleCollidingWithAABB() {
        Capsule capsule = new Capsule(new Vector3f(0, 0, 0), new Vector3f(0, 2, 0), 1.0f);
        AABB aabb = new AABB(new Vector3f(-1, -1, -1), new Vector3f(1, 1, 1));

        boolean result = cdSatGjk.isCollide(capsule, aabb);
        assertTrue(result, "Capsule and AABB should be colliding");
    }

    @Test
    void testCapsuleNotCollidingWithAABB() {
        Capsule capsule = new Capsule(new Vector3f(1, 1, 3), new Vector3f(1, 2, 3), 1.0f);
        AABB aabb = new AABB(new Vector3f(-1, -1, -1), new Vector3f(1, 1, 1));

        boolean result = cdSatGjk.isCollide(capsule, aabb);
        assertFalse(result, "Capsule and AABB should not be colliding");
    }

    @Test
    void testCylinderCollidingWithAABB() {
        Cylinder cylinder = new Cylinder(new Vector3f(0, 1, 0), 1.0f, 4.0f);
        AABB aabb = new AABB(new Vector3f(-1, 0, -1), new Vector3f(1, 3, 1));

        boolean result = cdSatGjk.isCollide(cylinder, aabb);
        assertTrue(result, "Cylinder and AABB should be colliding");
    }

    @Test
    void testCylinderNotCollidingWithAABB() {
        Cylinder cylinder = new Cylinder(new Vector3f(5, 0, 0), 1.0f, 2.0f);
        AABB aabb = new AABB(new Vector3f(-1, 0, -1), new Vector3f(1, 3, 1));

        boolean result = cdSatGjk.isCollide(cylinder, aabb);
        assertFalse(result, "Cylinder and AABB should not be colliding");
    }

    @Test
    void testCylinderCollidingWithPlane() {
        Cylinder cylinder = new Cylinder(new Vector3f(0, 1, 0), 1.0f, 2.0f);
        Plane plane = new Plane(new Vector3f(0, 1, 0), 0); // Plane with normal (0, 1, 0) and distance 0

        boolean result = cdSatGjk.isCollide(plane, cylinder);
        assertTrue(result, "Cylinder and Plane should be colliding");
    }

    @Test
    void testCylinderNotCollidingWithPlane() {
        Cylinder cylinder = new Cylinder(new Vector3f(0, 5, 0), 1.0f, 2.0f);
        Plane plane = new Plane(new Vector3f(0, 1, 0), 0); // Plane with normal (0, 1, 0) and distance 0

        boolean result = cdSatGjk.isCollide(plane, cylinder);
        assertFalse(result, "Cylinder and Plane should not be colliding");
    }

    @Test
    void testCapsuleCollidingWithPlane() {
        Capsule capsule = new Capsule(new Vector3f(0, 1, 0), new Vector3f(0, 3, 0), 1f);
        Plane plane = new Plane(new Vector3f(0, 1, 0), 2); // Plane with normal (0, 1, 0) and distance 2

        boolean result = cdSatGjk.isCollide(plane, capsule);
        assertTrue(result, "Capsule and Plane should be colliding");
    }

    @Test
    void testCapsuleNotCollidingWithPlane() {
        Capsule capsule = new Capsule(new Vector3f(0, 5, 0), new Vector3f(0, 7, 0), 0.5f);
        Plane plane = new Plane(new Vector3f(0, 1, 0), 0); // Plane with normal (0, 1, 0) and distance 0

        boolean result = cdSatGjk.isCollide(plane, capsule);
        assertFalse(result, "Capsule and Plane should not be colliding");
    }

    @Test
    void testCylinderCollidingWithTriangle() {
        Cylinder cylinder = new Cylinder(new Vector3f(0, 0, 0), 1.0f, 2.0f);

        Triangle triangle = new Triangle(new Vector3f(0, 0, 0), new Vector3f(1, 0, 0), new Vector3f(0, 1, 0));

        boolean collision = cdSatGjk.isCollide(cylinder, triangle);
        assertTrue(collision, "Expected collision between cylinder and triangle");
    }

    @Test
    void testCylinderNotCollidingWithTriangle() {
        Cylinder cylinder = new Cylinder(new Vector3f(5, 5, 5), 1.0f, 2.0f);

        Triangle triangle = new Triangle(new Vector3f(-3, 0, 0), new Vector3f(-2, 0, 0), new Vector3f(-2, -1, 0));

        boolean collision = cdSatGjk.isCollide(cylinder, triangle);
        assertFalse(collision, "Expected no collision between cylinder and triangle");
    }

    @Test
    void testCylinderCollidingWithOBB() {
        Cylinder cylinder = new Cylinder(new Vector3f(0, 0, 0), 1.0f, 2.0f);
        OBB obb = new OBB(new Vector3f(1, 0, 0), new Vector3f(1, 1, 1));

        boolean result = cdSatGjk.isCollide(cylinder, obb);
        assertTrue(result, "Collision should be detected");
    }

    @Test
    void testCylinderNotCollidingWithOBB() {
        Cylinder cylinder = new Cylinder(new Vector3f(0, 0, 0), 1.0f, 2.0f);
        OBB obb = new OBB(new Vector3f(4, 4, 4), new Vector3f(1, 1, 1));

        boolean result = cdSatGjk.isCollide(cylinder, obb);
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

        assertTrue(cdSatGjk.isCollide(plane, triangle));
    }

    @Test
    void testPlaneNotCollidingWithTriangle() {
        Vector3f normal = new Vector3f(0, 1, 0);
        Plane plane = new Plane(normal, 0);

        Vector3f vertex1 = new Vector3f(0, 0, 0);
        Vector3f vertex2 = new Vector3f(1, 0, 0);
        Vector3f vertex3 = new Vector3f(2, 0, 0);
        Triangle triangle = new Triangle(vertex1, vertex2, vertex3);

        assertFalse(cdSatGjk.isCollide(plane, triangle));
    }


    @Test
    public void testCapsuleCollidingWithOBB() {
        // Create a colliding capsule and OBB
        Capsule capsule = new Capsule(new Vector3f(0, 0, 0), new Vector3f(0, 1, 0), 0.5f);
        OBB obb = new OBB(new Vector3f(0, 0.5f, 0), new Vector3f(0.5f, 0.5f, 0.5f));

        assertTrue(cdSatGjk.isCollide(capsule, obb));
    }

    @Test
    public void testCapsuleNotCollidingWithOBB() {
        // Create a non-colliding capsule and OBB
        Capsule capsule = new Capsule(new Vector3f(3, 3, 0), new Vector3f(5, 5, 0), 0.5f);
        OBB obb = new OBB(new Vector3f(0, 0, 0), new Vector3f(0.5f, 0.5f, 0.5f));

        assertFalse(cdSatGjk.isCollide(capsule, obb));
    }

    @Test
    void testCapsuleCollidingWithTriangle() {
        Vector3f capsuleStart = new Vector3f(0, 0, 0);
        Vector3f capsuleEnd = new Vector3f(0, 2, 0);
        float capsuleRadius = 0.5f;
        Capsule capsule = new Capsule(capsuleStart, capsuleEnd, capsuleRadius);

        Vector3f vertex1 = new Vector3f(0, 0, 0);
        Vector3f vertex2 = new Vector3f(1, 0, 0);
        Vector3f vertex3 = new Vector3f(1, 1, 0);
        Triangle triangle = new Triangle(vertex1, vertex2, vertex3);

        assertTrue(cdSatGjk.isCollide(capsule, triangle));
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

        assertFalse(cdSatGjk.isCollide(capsule, triangle));
    }


    @Test
    void testNonCollidingPolyhedraSphere() {
        ConvexPolyhedron polyhedron = createNonCollidingPolyhedron1();
        Sphere sphere = new Sphere(new Vector3f(7, 7, 7), 1.0f);

        assertFalse(cdSatGjk.isCollide(sphere, polyhedron));
    }

    @Test
    void testCollidingPolyhedraSphere() {
        ConvexPolyhedron polyhedron = createNonCollidingPolyhedron1();
        Sphere sphere = new Sphere(new Vector3f(0, 0, 0), 1.0f);

        assertTrue(cdSatGjk.isCollide(sphere, polyhedron));
    }

    private ConvexPolyhedron createNonCollidingPolyhedron1() {
        List<Vector3f> vertices = new ArrayList<>();
        vertices.add(new Vector3f(0, 0, 0));
        vertices.add(new Vector3f(1, 0, 0));
        vertices.add(new Vector3f(1, 1, 0));
        vertices.add(new Vector3f(0, 1, 0));

        return new ConvexPolyhedron(vertices);
    }

    @Test
    public void CollidingPolyhedronCapsule() {
        // Create a capsule
        Vector3f capsuleStart = new Vector3f(0, 0, 0);
        Vector3f capsuleEnd = new Vector3f(0, 2, 0);
        float capsuleRadius = 1.0f;
        Capsule capsule = new Capsule(capsuleStart, capsuleEnd, capsuleRadius);

        // Create a convex polyhedron (e.g., a cube)
        List<Vector3f> vertices = new ArrayList<>();
        vertices.add(new Vector3f(0, 0, 0));
        vertices.add(new Vector3f(1, 0, 0));
        vertices.add(new Vector3f(1, 1, 0));
        vertices.add(new Vector3f(0, 1, 0));
        ConvexPolyhedron polyhedron = new ConvexPolyhedron(vertices);

        // Check for collision
        boolean collision = cdSatGjk.isCollide(capsule, polyhedron);

        // Assert that collision is detected
        assertTrue(collision);
    }

    @Test
    public void NonCollidingPolyhedronCapsule() {
        // Create a capsule
        Vector3f capsuleStart = new Vector3f(0, 8, 10);
        Vector3f capsuleEnd = new Vector3f(0, 12, 10);
        float capsuleRadius = 1.0f;
        Capsule capsule = new Capsule(capsuleStart, capsuleEnd, capsuleRadius);

        // Create a convex polyhedron (e.g., a cube)
        List<Vector3f> polyhedronVertices = new ArrayList<>();
        polyhedronVertices.add(new Vector3f(-2, -2, -2));
        polyhedronVertices.add(new Vector3f(2, -2, -2));
        polyhedronVertices.add(new Vector3f(2, 2, -2));
        polyhedronVertices.add(new Vector3f(-2, 2, -2));
        polyhedronVertices.add(new Vector3f(-2, -2, 2));
        polyhedronVertices.add(new Vector3f(2, -2, 2));
        polyhedronVertices.add(new Vector3f(2, 2, 2));
        polyhedronVertices.add(new Vector3f(-2, 2, 2));
        ConvexPolyhedron polyhedron = new ConvexPolyhedron(polyhedronVertices);

        // Check for collision
        boolean collision = cdSatGjk.isCollide(capsule, polyhedron);

        // Assert that no collision is detected
        assertFalse(collision);
    }


    @Test
    public void CollidingPolyhedronCylinder() {

        Cylinder cylinder = new Cylinder(new Vector3f(0, 0, 0), 1.0f, 2.0f);

        // Create a convex polyhedron (e.g., a cube)
        List<Vector3f> vertices = new ArrayList<>();
        vertices.add(new Vector3f(0, 0, 0));
        vertices.add(new Vector3f(1, 0, 0));
        vertices.add(new Vector3f(1, 1, 0));
        vertices.add(new Vector3f(0, 1, 0));
        ConvexPolyhedron polyhedron = new ConvexPolyhedron(vertices);

        // Check for collision
        boolean collision = cdSatGjk.isCollide(cylinder, polyhedron);

        // Assert that collision is detected
        assertTrue(collision);
    }

    @Test
    public void NonCollidingPolyhedronCylinder() {

        Cylinder cylinder = new Cylinder(new Vector3f(5, 5, 0), 1.0f, 2.0f);

        // Create a convex polyhedron (e.g., a cube)
        List<Vector3f> vertices = new ArrayList<>();
        vertices.add(new Vector3f(0, 0, 0));
        vertices.add(new Vector3f(1, 0, 0));
        vertices.add(new Vector3f(1, 1, 0));
        vertices.add(new Vector3f(0, 1, 0));
        ConvexPolyhedron polyhedron = new ConvexPolyhedron(vertices);

        // Check for collision
        boolean collision = cdSatGjk.isCollide(cylinder, polyhedron);

        // Assert that collision is detected
        assertFalse(collision);
    }


}