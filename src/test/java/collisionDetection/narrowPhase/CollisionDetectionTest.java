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