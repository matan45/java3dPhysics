package collisionDetection.primitive;

import org.joml.Vector3f;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class CollisionUtilsTest {

    @Test
    void testSphereInsideAABB() {
        Vector3f min = new Vector3f(-1, -1, -1);
        Vector3f max = new Vector3f(1, 1, 1);
        AABB aabb = new AABB(min, max);

        Sphere sphere = new Sphere(new Vector3f(0, 0, 0), 0.5f);

        CollisionResult expected = new CollisionResult(true, 0.0, new Vector3f(0, 0, 0), new Vector3f(0, 0, 0));
        CollisionResult result = CollisionUtils.isAABBCollidingWithSphere(aabb, sphere);

        assertEquals(expected, result);
    }

    @Test
    void testAABBCollidingWithSphere() {
        Vector3f min = new Vector3f(-1, -1, -1);
        Vector3f max = new Vector3f(1, 1, 1);
        AABB aabb = new AABB(min, max);

        Sphere sphere3 = new Sphere(new Vector3f(0, 1.5f, 0), 1.0f);

        CollisionResult expected = new CollisionResult(true, 0.5, new Vector3f(0, -0.5f, 0), new Vector3f(0, 1, 0));
        CollisionResult result = CollisionUtils.isAABBCollidingWithSphere(aabb, sphere3);

        assertEquals(expected, result);
    }

    @Test
    void testAABBNotCollidingWithSphere() {
        Vector3f min = new Vector3f(-1, -1, -1);
        Vector3f max = new Vector3f(1, 1, 1);
        AABB aabb = new AABB(min, max);

        Sphere sphere = new Sphere(new Vector3f(2, 2, 2), 0.5f);

        CollisionResult expected = new CollisionResult(false, 1.7320507764816284, new Vector3f(-0.5f, -0.5f, -0.5f), new Vector3f(1.5f, 1.5f, 1.5f));
        CollisionResult result = CollisionUtils.isAABBCollidingWithSphere(aabb, sphere);

        assertEquals(expected, result);
    }
}
