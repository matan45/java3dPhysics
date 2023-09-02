package collisionDetection.narrowPhase;

import collisionDetection.primitive.*;
import math.Vector3f;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

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
    public void testSphereCollisionLine(){
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
    public void testCylinderCollisionLine(){
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
    public void testPlaneCollisionLine(){
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
    public void testOBBCollisionLine(){
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
    public void testTriangleCollisionLine(){
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
    public void testTerrainCollisionLine(){
        // Create a TerrainShape with height data, borders, width, and length for testing collision
        // Sample height data for a 3x3 grid (for demonstration purposes)
        float[][] heightData = {
                {0, 0, 0},
                {0, 2, 0},
                {0, 0, 0}
        };

        // Define a simple AABB for terrain borders
        AABB borders = new AABB(new Vector3f(-1, -1, -1), new Vector3f(5, 5, 5));

        // Initialize the TerrainShape object
        TerrainShape terrainShape = new TerrainShape(heightData, borders,new Vector3f(), 3, 3);

        // Create a Line object representing a line segment that collides with the terrain
        Line line = new Line(new Vector3f(0, 5, 0), new Vector3f(0, -1, 0));

        // Assert that the result is true since the line should collide with the terrain
        assertTrue(CollisionDetection.isCollide(line, terrainShape));

        // Create a Line object representing a line segment that does not collide with the terrain
        Line line2 = new Line(new Vector3f(3, 3, 3), new Vector3f(4, 4, 4));

        // Assert that the result is false since the line should not collide with the terrain
        assertFalse(CollisionDetection.isCollide(line2, terrainShape));
    }

}