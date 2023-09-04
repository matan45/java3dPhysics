package collisionDetection.primitive;

import math.Vector3f;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class AABBTest {

    @Test
    public void testIsPointInside() {
        // Create an AABB instance for testing
        Vector3f min = new Vector3f(0, 0, 0);
        Vector3f max = new Vector3f(5, 5, 5);
        AABB aabb = new AABB(min, max);

        // Test points
        Vector3f insidePoint = new Vector3f(2, 2, 2);
        Vector3f outsidePoint = new Vector3f(6, 6, 6);

        assertTrue(aabb.isPointInside(insidePoint));
        assertFalse(aabb.isPointInside(outsidePoint));
    }

    @Test
    public void testClosestPoint() {
        // Create an AABB instance for testing
        Vector3f min = new Vector3f(0, 0, 0);
        Vector3f max = new Vector3f(5, 5, 5);
        AABB aabb = new AABB(min, max);

        // Test point
        Vector3f testPoint = new Vector3f(2, 7, 2);

        Vector3f closestPoint = aabb.closestPoint(testPoint);
        // Calculate the expected closest point manually or using your own method
        Vector3f expectedClosestPoint = new Vector3f(2, 5, 2);

        assertEquals(expectedClosestPoint, closestPoint);
    }


}