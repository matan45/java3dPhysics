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
    @Test
    public void testIntersects() {
        OBB obb1 = new OBB(
                new Vector3f(0.0f, 3.0f, 0.0f),
                new Vector3f(2.0f, 2.0f, 2.0f)
        );

        OBB obb2 = new OBB(
                new Vector3f(0.0f, 4.0f, 0.0f),
                new Vector3f(2.0f, 2.0f, 2.0f)
        );

        assertTrue(OBB.isOBBColliding(obb1, obb2),"OBBs should be colliding");
    }

    @Test
    public void testNonCollidingOBBs() {
        Vector3f center1 = new Vector3f(0.0f, 0.0f, 0.0f);
        Vector3f halfExtents1 = new Vector3f(1.0f, 1.0f, 1.0f);
        OBB obb1 = new OBB(center1, halfExtents1);

        Vector3f center2 = new Vector3f(3.0f, 3.0f, 3.0f);
        Vector3f halfExtents2 = new Vector3f(1.0f, 1.0f, 1.0f);
        OBB obb2 = new OBB(center2, halfExtents2);

        boolean collision = OBB.isOBBColliding(obb1, obb2);

        assertFalse(collision);
    }

    @Test
    public void testNonParallelEdges() {
        // Create two non-parallel OBBs
        OBB obb1 = new OBB(new Vector3f(0, 0, 0),new Vector3f(1, 1, 1));
        OBB obb2 = new OBB(new Vector3f(4, 4, 4),new Vector3f(1, 1, 1));

        assertFalse(OBB.isOBBColliding(obb1, obb2), "Non-parallel OBBs should not be colliding");
    }
}
