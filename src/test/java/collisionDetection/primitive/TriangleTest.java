package collisionDetection.primitive;

import math.Vector3f;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class TriangleTest {

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