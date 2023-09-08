package collisionDetection.narrowPhase.sat;


import collisionDetection.primitive.*;
import math.Vector3f;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;



class SATTest {

    @Test
    public void testOBBCollidingWithAABB() {
        Vector3f min = new Vector3f(-1.0f, -1.0f, -1.0f);
        Vector3f max = new Vector3f(1.0f, 1.0f, 1.0f);
        AABB aabb = new AABB(min, max);

        Vector3f center = new Vector3f(0.0f, 0.0f, 0.0f);
        Vector3f halfExtents = new Vector3f(1.0f, 1.0f, 1.0f);
        OBB obb = new OBB(center, halfExtents);

        boolean collision = SAT.isCollide(aabb, obb);

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

        boolean collision = SAT.isCollide(aabb, obb);

        assertFalse(collision,"OBB AABB should not be colliding");
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

        assertTrue(SAT.isCollide(triangle, aabb));
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

        assertFalse(SAT.isCollide(triangle, aabb));
    }


    @Test
    void testNonCollidingPolyhedra() {
        ConvexPolyhedron polyhedron1 = createNonCollidingPolyhedron1();
        ConvexPolyhedron polyhedron2 = createNonCollidingPolyhedron2();

        assertFalse(SAT.isCollide(polyhedron1, polyhedron2));
    }

    @Test
    void testCollidingPolyhedra() {
        ConvexPolyhedron polyhedron1 = createCollidingPolyhedron1();
        ConvexPolyhedron polyhedron2 = createCollidingPolyhedron2();

        assertTrue(SAT.isCollide(polyhedron1, polyhedron2));
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

        assertTrue(SAT.isCollide(obb1, obb2),"OBBs should be colliding");
    }

    @Test
    public void testNonCollidingOBBs() {
        Vector3f center1 = new Vector3f(0.0f, 0.0f, 0.0f);
        Vector3f halfExtents1 = new Vector3f(1.0f, 1.0f, 1.0f);
        OBB obb1 = new OBB(center1, halfExtents1);

        Vector3f center2 = new Vector3f(3.0f, 3.0f, 3.0f);
        Vector3f halfExtents2 = new Vector3f(1.0f, 1.0f, 1.0f);
        OBB obb2 = new OBB(center2, halfExtents2);

        boolean collision = SAT.isCollide(obb1, obb2);

        assertFalse(collision);
    }

    @Test
    public void testNonParallelEdgesOBB() {
        // Create two non-parallel OBBs
        OBB obb1 = new OBB(new Vector3f(0, 0, 0),new Vector3f(1, 1, 1));
        OBB obb2 = new OBB(new Vector3f(4, 4, 4),new Vector3f(1, 1, 1));

        assertFalse(SAT.isCollide(obb1, obb2), "Non-parallel OBBs should not be colliding");
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
        assertTrue(SAT.isCollide(triangle1, triangle2),"Triangles should be colliding");
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

        assertFalse(SAT.isCollide(triangle1, triangle2),"Triangles should not be colliding");
    }

    @Test
    public void  parallelEdgesTriangle() {
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

        assertFalse(SAT.isCollide(triangle1, triangle2),"Triangles not be colliding");
    }

    @Test
    void testIsAABBCollidingWithAABB() {
        Vector3f min1 = new Vector3f(0, 0, 0);
        Vector3f max1 = new Vector3f(2, 2, 2);
        AABB aabb1 = new AABB(min1, max1);

        Vector3f min2 = new Vector3f(1, 1, 1);
        Vector3f max2 = new Vector3f(3, 3, 3);
        AABB aabb2 = new AABB(min2, max2);

        boolean result = SAT.isCollide(aabb1, aabb2);

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

        boolean result = SAT.isCollide(aabb1, aabb2);

        assertFalse(result, "AABBs should not be colliding");
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
        assertTrue(SAT.isCollide(line1, cube));

        Line line2 = new Line(new Vector3f(0, 0, 0), new Vector3f(0, 0, 1));
        assertTrue(SAT.isCollide(line2, cube));

        Line line3 = new Line(new Vector3f(-2, -2, -2), new Vector3f(1, 1, 1));
        assertTrue(SAT.isCollide(line3, cube));

        // Test cases where the line does not collide with the cube
        Line line4 = new Line(new Vector3f(2, 2, 2), new Vector3f(4, 4, 4));
        assertFalse(SAT.isCollide(line4, cube));

        Line line5 = new Line(new Vector3f(2, 2, 2), new Vector3f(3, 3, 3));
        assertFalse(SAT.isCollide(line5, cube));

        Line line6 = new Line(new Vector3f(2, 2, 2), new Vector3f(2, 2, 3));
        assertFalse(SAT.isCollide(line6, cube));
    }

    @Test
    public void testAABBCollisionLine() {
        // Create an AABB with minimum corner (-1, -1, -1) and maximum corner (1, 1, 1)
        AABB aabb = new AABB(new Vector3f(-1, -1, -1), new Vector3f(1, 1, 1));

        // Test cases where the line intersects the AABB
        Line line1 = new Line(new Vector3f(0, 0, -2), new Vector3f(0, 0, 2));
        assertTrue(SAT.isCollide(line1, aabb));

        Line line2 = new Line(new Vector3f(-2, -2, -2), new Vector3f(-1, -1, -1));
        assertTrue(SAT.isCollide(line2, aabb));

        Line line3 = new Line(new Vector3f(0, 0, 0), new Vector3f(0, 0, 1));
        assertTrue(SAT.isCollide(line3, aabb));

        // Test cases where the line does not intersect the AABB
        Line line4 = new Line(new Vector3f(2, 2, 2), new Vector3f(3, 3, 3));
        assertFalse(SAT.isCollide(line4, aabb));

        Line line5 = new Line(new Vector3f(2, 2, 2), new Vector3f(2, 2, 3));
        assertFalse(SAT.isCollide(line5, aabb));

        Line line6 = new Line(new Vector3f(2, 0, 0), new Vector3f(3, 0, 0));
        assertFalse(SAT.isCollide(line6, aabb));
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
        assertTrue(SAT.isCollide(line, triangle));

        // Test non-intersecting case
        Line nonIntersectingLine = new Line(new Vector3f(3, 3, 3), new Vector3f(4, 4, 4));
        assertFalse(SAT.isCollide(nonIntersectingLine, triangle));

        Line line2 = new Line(new Vector3f(0.5f, 0.5f, 0), new Vector3f(1.5f, 1.5f, 0));

        assertTrue(SAT.isCollide(line2, triangle));

        // Create a triangle entirely inside a line segment
        vertex1 = new Vector3f(0, 0, 0);
        vertex2 = new Vector3f(4, 0, 0);
        vertex3 = new Vector3f(4, 4, 0);

        Triangle triangle2 = new Triangle(vertex1, vertex2, vertex3);
        Line line3 = new Line(new Vector3f(1, 1, 0), new Vector3f(5, 5, 0));

        assertTrue(SAT.isCollide(line3, triangle2));
    }

    @Test
    public void testOBBCollisionLine() {
        // Create an OBB with a known center and half extents
        OBB obb = new OBB(new Vector3f(1, 1, 1), new Vector3f(1, 1, 1));

        // Test a line that intersects with the OBB
        Line intersectingLine = new Line(new Vector3f(0, 0, 0), new Vector3f(2, 2, 2));
        assertTrue(SAT.isCollide(intersectingLine, obb));

        // Test a line that does not intersect with the OBB
        Line nonIntersectingLine = new Line(new Vector3f(3, 3, 0), new Vector3f(5, 5, 0));
        assertFalse(SAT.isCollide(nonIntersectingLine, obb));

        // Create an OBB
        OBB obb2 = new OBB(new Vector3f(0.0f, 0.0f, 0.0f), new Vector3f(1.0f, 1.0f, 1.0f));

        // Create a line that intersects with the OBB
        Line line1 = new Line(new Vector3f(-2.0f, 0.0f, 0.0f), new Vector3f(2.0f, 0.0f, 0.0f));

        // Create a line that doesn't intersect with the OBB
        Line line2 = new Line(new Vector3f(-2.0f, 2.0f, 0.0f), new Vector3f(-1.0f, 2.0f, 0.0f));

        assertTrue(SAT.isCollide(line1, obb2)); // Expect collision
        assertFalse(SAT.isCollide(line2, obb2)); // Expect no collision
    }

    @Test
    public void testLineCollision() {
        // Test cases where the lines intersect
        Line line1 = new Line(new Vector3f(0, 0, 0), new Vector3f(1, 1, 0));
        Line line2 = new Line(new Vector3f(0, 1, 0), new Vector3f(1, 0, 0));
        assertTrue(SAT.isCollide(line1, line2));

        Line line3 = new Line(new Vector3f(0, 0, 0), new Vector3f(1, 1, 0));
        Line line4 = new Line(new Vector3f(0.5f, 0.5f, 0), new Vector3f(1, 0, 0));
        assertTrue(SAT.isCollide(line3, line4));

        // Test cases where the lines do not intersect
        Line line5 = new Line(new Vector3f(0, 0, 0), new Vector3f(1, 1, 0));
        Line line6 = new Line(new Vector3f(2, 2, 0), new Vector3f(3, 3, 0));
        assertFalse(SAT.isCollide(line5, line6));

        Line line7 = new Line(new Vector3f(0, 0, 0), new Vector3f(1, 1, 0));
        Line line8 = new Line(new Vector3f(0, 2, 0), new Vector3f(1, 3, 0));
        assertFalse(SAT.isCollide(line7, line8));

        // Test cases where the lines are parallel but not collinear
        Line line9 = new Line(new Vector3f(0, 0, 0), new Vector3f(1, 1, 0));
        Line line10 = new Line(new Vector3f(0, 0, 1), new Vector3f(1, 1, 1));
        assertFalse(SAT.isCollide(line9, line10));

        Line line11 = new Line(new Vector3f(2, 2, 2), new Vector3f(4, 4, 4));
        Line line12 = new Line(new Vector3f(1, -1, 1), new Vector3f(1, 1, 1));
        assertFalse(SAT.isCollide(line11, line12));
    }


}