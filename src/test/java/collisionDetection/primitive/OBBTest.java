package collisionDetection.primitive;

import org.joml.Vector3f;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class OBBTest {

    @Test
    void testIsOBBCollidingWithOBB() {
        Vector3f center1 = new Vector3f(0, 0, 0);
        Vector3f[] axes1 = {new Vector3f(1, 0, 0), new Vector3f(0, 1, 0), new Vector3f(0, 0, 1)};
        float[] halfLengths1 = {0.3f, 0.3f, 0.3f};
        OBB obb1 = new OBB(center1, axes1, halfLengths1);

        Vector3f center2 = new Vector3f(0.4f, 0.4f, 0.4f);
        Vector3f[] axes2 = {new Vector3f(1, 0, 0), new Vector3f(0, 1, 0), new Vector3f(0, 0, 1)};
        float[] halfLengths2 = {0.1f, 0.1f, 0.1f};
        OBB obb2 = new OBB(center2, axes2, halfLengths2);

        CollisionResult expected = new CollisionResult(true, 0.0, new Vector3f(0.3f, 0.3f, 0.3f), new Vector3f(0.3f, 0.3f, 0.3f));
        CollisionResult result = OBB.isOBBCollidingWithOBB(obb1, obb2);

        assertEquals(expected, result);
    }

    @Test
    void testIsOBBNoCollidingWithOBB() {
        Vector3f center1 = new Vector3f(0, 0, 0);
        float[] halfLengths1 = {1, 1, 1};
        Vector3f[] axes1 = {new Vector3f(1, 0, 0), new Vector3f(0, 1, 0), new Vector3f(0, 0, 1)};
        OBB obb1 = new OBB(center1, axes1, halfLengths1);

        Vector3f center2 = new Vector3f(3, 3, 3);
        float[] halfLengths2 = {1, 1, 1};
        Vector3f[] axes2 = {new Vector3f(1, 0, 0), new Vector3f(0, 1, 0), new Vector3f(0, 0, 1)};
        OBB obb2 = new OBB(center2, axes2, halfLengths2);

        CollisionResult expected = new CollisionResult(false, 1.7320508075688772, new Vector3f(1, 1, 1), new Vector3f(0, 0, 0));
        CollisionResult result = OBB.isOBBCollidingWithOBB(obb1, obb2);

        assertEquals(expected, result);
    }
}
