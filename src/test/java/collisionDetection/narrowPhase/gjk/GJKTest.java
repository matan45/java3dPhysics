package collisionDetection.narrowPhase.gjk;

import collisionDetection.primitive.AABB;
import math.Vector3f;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class GJKTest {

    @Test
    void testIsAABBCollidingWithAABB() {
        Vector3f min1 = new Vector3f(0, 0, 0);
        Vector3f max1 = new Vector3f(2, 2, 0);
        AABB aabb1 = new AABB(min1, max1);

        Vector3f min2 = new Vector3f(1, 1, 0);
        Vector3f max2 = new Vector3f(3, 3, 0);
        AABB aabb2 = new AABB(min2, max2);

        boolean result = GJK.collision(aabb1, aabb2);

        assertTrue(result, "AABBs should be colliding");

    }

}