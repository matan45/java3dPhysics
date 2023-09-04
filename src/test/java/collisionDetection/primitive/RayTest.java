package collisionDetection.primitive;

import math.Vector3f;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class RayTest {
    @Test
    public void testClosestPoint() {
        // Create a ray with an origin at (0, 0, 0) and a direction along the positive X-axis.
        Ray ray = new Ray(new Vector3f(0, 0, 0), new Vector3f(1, 0, 0));

        // Test case 1: The closest point to (2, 0, 0) should be (2, 0, 0) itself.
        Vector3f closest1 = ray.closestPoint(new Vector3f(2, 0, 0));
        assertEquals(new Vector3f(2, 0, 0), closest1);

        // Test case 2: The closest point to (0, 1, 0) should be (0, 0, 0) since it's on the ray.
        Vector3f closest2 = ray.closestPoint(new Vector3f(0, 1, 0));
        assertEquals(new Vector3f(0, 0, 0), closest2);

        // Test case 3: The closest point to (-1, 0, 0) should be (0, 0, 0) since it's on the ray.
        Vector3f closest3 = ray.closestPoint(new Vector3f(-1, 0, 0));
        assertEquals(new Vector3f(0, 0, 0), closest3);

        // Test case 4: The closest point to (0, 0, 1) should be (0, 0, 0) since it's on the ray.
        Vector3f closest4 = ray.closestPoint(new Vector3f(0, 0, 1));
        assertEquals(new Vector3f(0, 0, 0), closest4);

        // Test case 5: The closest point to a point behind the ray (e.g., (-1, 0, 0))
        // should be the origin of the ray itself (0, 0, 0).
        Vector3f closest5 = ray.closestPoint(new Vector3f(-1, 0, 0));
        assertEquals(new Vector3f(0, 0, 0), closest5);
    }

    @Test
    public void testIsPointInside() {
        // Create a ray with origin at (0, 0, 0) and direction along the positive X-axis.
        Ray ray = new Ray(new Vector3f(0, 0, 0), new Vector3f(1, 0, 0));

        // Test case 1: A point on the ray should be considered inside.
        assertTrue(ray.isPointInside(new Vector3f(1, 0, 0)));

        // Test case 2: A point in the same direction as the ray should be inside.
        assertTrue(ray.isPointInside(new Vector3f(2, 0, 0)));

        // Test case 3: A point in the opposite direction of the ray should not be inside.
        assertFalse(ray.isPointInside(new Vector3f(-1, 0, 0)));

        // Test case 4: A point in a different direction but on the same plane as the ray should not be inside.
        assertFalse(ray.isPointInside(new Vector3f(0, 1, 0)));

        // Test case 4: A point in a different direction but on the same plane as the ray should not be inside.
        assertFalse(ray.isPointInside(new Vector3f(0, 0, 1)));

    }
}