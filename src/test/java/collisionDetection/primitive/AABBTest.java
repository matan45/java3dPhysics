package collisionDetection.primitive;

import org.joml.Vector3f;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class AABBTest {

    @Test
    void testAABBClosestPointPair() {
        Vector3f min1 = new Vector3f(0, 0, 0);
        Vector3f max1 = new Vector3f(2, 2, 2);
        AABB aabb1 = new AABB(min1, max1);

        Vector3f min2 = new Vector3f(1, 1, 1);
        Vector3f max2 = new Vector3f(3, 3, 3);
        AABB aabb2 = new AABB(min2, max2);

        CollisionResult expected = new CollisionResult(true, 1.0, new Vector3f(), new Vector3f(0,0,1));
        CollisionResult result = AABB.isAABBColliding(aabb1, aabb2);

        assertEquals(expected, result, "AABBs should be colliding");

    }

    @Test
    void testAABBClosestPointPairNotCollide() {
        Vector3f min1 = new Vector3f(0, 0, 0);
        Vector3f max1 = new Vector3f(2, 2, 2);
        AABB aabb1 = new AABB(min1, max1);

        Vector3f min2 = new Vector3f(3, 3, 3);
        Vector3f max2 = new Vector3f(5, 5, 5);
        AABB aabb2 = new AABB(min2, max2);

        CollisionResult expected = new CollisionResult(false, 1.0, new Vector3f(2, 2, 2), new Vector3f(2, 2, 3));
        CollisionResult result = AABB.isAABBColliding(aabb1, aabb2);

        assertEquals(expected, result, "AABBs should not be colliding");
    }
}