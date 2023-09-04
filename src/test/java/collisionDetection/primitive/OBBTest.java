package collisionDetection.primitive;

import math.Vector3f;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class OBBTest {

    @Test
    void testIsPointInside() {
        Vector3f center = new Vector3f(0.0f, 0.0f, 0.0f);
        Vector3f halfExtents = new Vector3f(1.0f, 1.0f, 1.0f);
        OBB obb = new OBB(center, halfExtents);

        // Test a point inside the bounding box
        Vector3f insidePoint = new Vector3f(0.5f, 0.5f, 0.5f);
        assertTrue(obb.isPointInside(insidePoint));

        // Test a point outside the bounding box
        Vector3f outsidePoint = new Vector3f(2.0f, 2.0f, 2.0f);
        assertFalse(obb.isPointInside(outsidePoint));
    }

    @Test
    void testClosestPoint() {
        Vector3f center = new Vector3f(0.0f, 0.0f, 0.0f);
        Vector3f halfExtents = new Vector3f(1.0f, 1.0f, 1.0f);
        OBB obb = new OBB(center, halfExtents);

        Vector3f point = new Vector3f(2.0f, 2.0f, 2.0f);
        Vector3f closest = obb.closestPoint(point);

        // Assert that the closest point is within the bounding box
        assertEquals(new Vector3f(1, 1, 1),closest);
    }

}
