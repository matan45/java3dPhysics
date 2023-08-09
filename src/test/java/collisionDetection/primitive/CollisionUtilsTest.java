package collisionDetection.primitive;

import org.joml.Vector3f;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class CollisionUtilsTest {


    @Test
    void testAABBCollidingWithSphere() {
        Vector3f min = new Vector3f(-1, -1, -1);
        Vector3f max = new Vector3f(1, 1, 1);
        AABB aabb = new AABB(min, max);

        Sphere sphere3 = new Sphere(new Vector3f(0, 1.5f, 0), 1.0f);

        boolean result = CollisionUtils.isAABBCollidingWithSphere(aabb, sphere3);

        assertTrue(result, "AABB Sphere should be colliding");
    }

    @Test
    void testAABBNotCollidingWithSphere() {
        Vector3f min = new Vector3f(-1, -1, -1);
        Vector3f max = new Vector3f(1, 1, 1);
        AABB aabb = new AABB(min, max);

        Sphere sphere = new Sphere(new Vector3f(2, 2, 2), 0.5f);
       boolean result = CollisionUtils.isAABBCollidingWithSphere(aabb, sphere);

        assertFalse(result, "AABB Sphere should not be colliding");
    }
}
