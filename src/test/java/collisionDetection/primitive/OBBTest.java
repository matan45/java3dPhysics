package collisionDetection.primitive;

import org.joml.Vector3f;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class OBBTest {
    @Test
    public void testIntersects() {
        OBB obb1 = new OBB(
                new Vector3f(2.0f, 3.0f, 1.0f),
                new Vector3f(2.0f, 1.0f, 0.5f)
        );

        OBB obb2 = new OBB(
                new Vector3f(3.5f, 4.5f, 1.25f),
                new Vector3f(1.5f, 0.8f, 0.3f)
        );

        assertTrue(OBB.isOBBColliding(obb1, obb2),"OBBs should be colliding");
    }

    @Test
    public void testDoesNotIntersect() {
        OBB obb1 = new OBB(
                new Vector3f(2.0f, 3.0f, 1.0f),
                new Vector3f(2.0f, 1.0f, 0.5f)
        );

        OBB obb2 = new OBB(
                new Vector3f(8f, 7f, 4f),
                new Vector3f(1.5f, 0.8f, 0.3f)
        );

        assertFalse(OBB.isOBBColliding(obb1, obb2),"OBBs should not be colliding");
    }
}
