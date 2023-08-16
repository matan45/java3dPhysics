package collisionDetection.primitive;

import org.joml.Vector3f;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class RayTest {
    @Test
    public void testRayIntersectsSphere() {
        Ray ray = new Ray(new Vector3f(0, 0, 0), new Vector3f(1, 0, 0)); // Example ray
        Sphere sphere = new Sphere(new Vector3f(2, 0, 0), 1.0f); // Example sphere

        assertTrue(Ray.isSphereCollide(ray, sphere));
    }

    @Test
    public void testRayMissesSphere() {
        Ray ray = new Ray(new Vector3f(0, 0, 0), new Vector3f(0, 1, 0)); // Example ray
        Sphere sphere = new Sphere(new Vector3f(3, 0, 0), 1.0f); // Example sphere

        assertFalse(Ray.isSphereCollide(ray, sphere));
    }


    @Test
    public void testRayIntersectsABB() {
        // Create an AABB
        AABB aabb = new AABB(new Vector3f(0, 0, 0), new Vector3f(2, 2, 2));

        // Create a ray
        Ray ray = new Ray(new Vector3f(1, 1, 5), new Vector3f(0, 0, -1));

        // Test for collision
        assertTrue(Ray.isAABBCollide(ray, aabb));
    }

    @Test
    public void testRayMissesAABB() {
        // Create an AABB
        AABB aabb = new AABB(new Vector3f(0, 0, 0), new Vector3f(2, 2, 2));

        // Create a ray
        Ray ray = new Ray(new Vector3f(1, 1, 5), new Vector3f(0, -1, 0));

        // Test for no collision
        assertFalse(Ray.isAABBCollide(ray, aabb));
    }

    @Test
    public void testRayIntersectsCapsule() {
        // Create a capsule
        Capsule capsule = new Capsule(new Vector3f(0, 0, 0), new Vector3f(0, 4, 0), 1.0f);

        // Create a ray
        Ray ray = new Ray(new Vector3f(0, -1, 0), new Vector3f(0, 1, 0));

        // Test for collision
        assertTrue(Ray.isCapsuleCollide(ray, capsule));
    }

    @Test
    public void testRayMissesCapsule() {
        // Create a capsule
        Capsule capsule = new Capsule(new Vector3f(0, 0, 0), new Vector3f(0, 4, 0), 1.0f);

        // Create a ray
        Ray ray = new Ray(new Vector3f(2, 2, 2), new Vector3f(0, 0, -1));

        // Test for no collision
        assertFalse(Ray.isCapsuleCollide(ray, capsule));
    }

}