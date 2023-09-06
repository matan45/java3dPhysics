package collisionDetection.narrowPhase.sat;

import collisionDetection.narrowPhase.gjk.GJK;
import collisionDetection.primitive.AABB;
import math.Vector3f;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class SATTest {

    @Test
    void testIsAABBCollidingWithAABB() {
        Vector3f min1 = new Vector3f(0, 0, 0);
        Vector3f max1 = new Vector3f(2, 2, 2);
        AABB aabb1 = new AABB(min1, max1);

        Vector3f min2 = new Vector3f(1, 1, 1);
        Vector3f max2 = new Vector3f(3, 3, 3);
        AABB aabb2 = new AABB(min2, max2);

        boolean result = SAT.collision(aabb1, aabb2);

        assertTrue(result, "AABBs should be colliding");

    }

    @Test
    void testIsAABBNotCollidingWithAABB() {
        Vector3f min1 = new Vector3f(-5, -5, 0);
        Vector3f max1 = new Vector3f(-1, -1, 0);
        AABB aabb1 = new AABB(min1, max1);

        Vector3f min2 = new Vector3f(1, 1, 0);
        Vector3f max2 = new Vector3f(3, 3, 0);
        AABB aabb2 = new AABB(min2, max2);

        boolean result = SAT.collision(aabb1, aabb2);

        assertFalse(result, "AABBs should not be colliding");

    }

}