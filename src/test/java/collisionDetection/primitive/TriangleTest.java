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
        assertEquals(new Vector3f(0.5f, 0.5f, 0), closest);
    }
}