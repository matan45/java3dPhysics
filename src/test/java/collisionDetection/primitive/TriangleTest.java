package collisionDetection.primitive;

import math.Vector3f;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class TriangleTest {

    @Test
    public void testIsPointInside() {
        Vector3f vertex1 = new Vector3f(0, 0, 0);
        Vector3f vertex2 = new Vector3f(0, 1, 0);
        Vector3f vertex3 = new Vector3f(1, 0, 0);
        Triangle triangle = new Triangle(vertex1, vertex2, vertex3);

        // Test points inside and outside the triangle
        assertTrue(triangle.isPointInside(new Vector3f(0.5f, 0.5f, 0)));
        assertFalse(triangle.isPointInside(new Vector3f(2f, 2f, 0)));
    }

    @Test
    public void testClosestPoint() {
        Vector3f vertex1 = new Vector3f(0, 0, 0);
        Vector3f vertex2 = new Vector3f(0, 1, 0);
        Vector3f vertex3 = new Vector3f(1, 0, 0);
        Triangle triangle = new Triangle(vertex1, vertex2, vertex3);

        Vector3f closest = triangle.closestPoint(new Vector3f(2f, 2f, 0));
        assertEquals(new Vector3f(0.5f, 0.5f, 0), closest); // This is just an example, update with the actual expected result
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
        assertTrue(Triangle.isTriangleColliding(triangle1, triangle2),"Triangles should be colliding");
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

        assertFalse(Triangle.isTriangleColliding(triangle1, triangle2),"Triangles should not be colliding");
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

        assertFalse(Triangle.isTriangleColliding(triangle1, triangle2),"Triangles not be colliding");
    }

}