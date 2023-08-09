package collisionDetection.primitive;

import org.joml.Vector3f;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class TriangleTest {

    @Test
    public void testIntersects() {
        Triangle triangle1 = new Triangle(
                new Vector3f(1.0f, 1.0f, 1.0f),
                new Vector3f(2.0f, 2.0f, 1.0f),
                new Vector3f(1.0f, 2.0f, 1.0f)
        );

        Triangle triangle2 = new Triangle(
                new Vector3f(1.5f, 1.5f, 0.5f),
                new Vector3f(2.5f, 2.5f, 0.5f),
                new Vector3f(1.5f, 2.5f, 0.5f)
        );

        assertTrue(Triangle.isTriangleColliding(triangle1, triangle2),"Triangles should be colliding");
    }

    @Test
    public void testDoesNotIntersect() {
        Triangle triangle1 = new Triangle(
                new Vector3f(1.0f, 1.0f, 1.0f),
                new Vector3f(2.0f, 2.0f, 1.0f),
                new Vector3f(1.0f, 2.0f, 1.0f)
        );

        Triangle triangle2 = new Triangle(
                new Vector3f(8f, 8f, 8f),
                new Vector3f(6f, 6f, 6f),
                new Vector3f(9f, 9f, 9f)
        );


        assertFalse(Triangle.isTriangleColliding(triangle1, triangle2),"Triangles should not be colliding");
    }

}