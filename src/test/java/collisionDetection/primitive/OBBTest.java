package collisionDetection.primitive;

import org.joml.Vector3f;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class OBBTest {

    @Test
    void testIsOBBCollidingWithOBBCollision() {
        Vector3f center1 = new Vector3f(0, 0, 0);
        float[] halfLengths1 = { 1, 1, 1 };
        OBB obb1 = new OBB(center1, halfLengths1);

        Vector3f center2 = new Vector3f(1, 1, 1);
        float[] halfLengths2 = { 1, 1, 1 };
        OBB obb2 = new OBB(center2,  halfLengths2);

        CollisionResult expected = new CollisionResult(false, 3.1961522102355957, new Vector3f(2.0773504f, 2.0773504f, 2.0773504f), new Vector3f(0, 0, 0));
        CollisionResult result = OBB.isOBBCollidingWithOBB(obb1, obb2);

        assertEquals(expected, result);
    }

    @Test
    void testIsOBBCollidingWithOBBNoCollision() {
        Vector3f center1 = new Vector3f(0, 0, 0);
        float[] halfLengths1 = { 1, 1, 1 };
        OBB obb1 = new OBB(center1,  halfLengths1);

        Vector3f center2 = new Vector3f(3, 3, 3);
        float[] halfLengths2 = { 1, 1, 1 };
        OBB obb2 = new OBB(center2, halfLengths2);

        CollisionResult expected = new CollisionResult(false, 3.1961522102355957, new Vector3f(2.0773504f, 2.0773504f, 2.0773504f), new Vector3f(0, 0, 0));
        CollisionResult result = OBB.isOBBCollidingWithOBB(obb1, obb2);

        assertEquals(expected, result);
    }
}
