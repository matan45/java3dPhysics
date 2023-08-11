package collisionDetection.primitive;

import org.joml.Vector3f;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class OBBTest {
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
}
