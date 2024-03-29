package collisionDetection.primitive;

import math.Vector3f;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class CapsuleTest {

    @Test
    public void testIsPointInside() {
        // Create a Capsule instance for testing
        Vector3f start = new Vector3f(0, 0, 0);
        Vector3f end = new Vector3f(0, 5, 0);
        float radius = 2.0f;
        Capsule capsule = new Capsule(start, end, radius);

        // Test points
        Vector3f insidePoint = new Vector3f(0, 2, 0);
        Vector3f outsidePoint = new Vector3f(3, 7, 0);

        assertTrue(capsule.isPointInside(insidePoint));
        assertFalse(capsule.isPointInside(outsidePoint));
    }

    @Test
    public void testClosestPoint() {
        // Create a Capsule instance for testing
        Vector3f start = new Vector3f(0, 0, 0);
        Vector3f end = new Vector3f(0, 5, 0);
        float radius = 2.0f;
        Capsule capsule = new Capsule(start, end, radius);

        // Test point
        Vector3f testPoint = new Vector3f(0, 7, 0);

        Vector3f closestPoint = capsule.closestPoint(testPoint);
        // Calculate the expected closest point manually or using your own method
        Vector3f expectedClosestPoint = new Vector3f(0.0f, 7.0f, 0.0f);

        assertEquals(expectedClosestPoint, closestPoint);
    }

}