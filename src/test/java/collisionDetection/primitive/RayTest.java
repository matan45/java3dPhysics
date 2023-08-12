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

}