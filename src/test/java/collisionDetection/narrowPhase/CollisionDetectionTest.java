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

}