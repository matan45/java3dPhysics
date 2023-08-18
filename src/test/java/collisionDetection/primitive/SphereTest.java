package collisionDetection.primitive;

import math.Vector3f;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class SphereTest {


    @Test
    void testIsSphereCollidingWithSphere() {
        Sphere sphere1 = new Sphere(new Vector3f(0, 0, 0), 1.0f);
        Sphere sphere2 = new Sphere(new Vector3f(1, 1, 1), 1.0f);

        boolean result = Sphere.isSphereColliding(sphere1, sphere2);

        assertTrue(result, "Spheres should be colliding");
    }

    @Test
    void testIsSphereNoCollidingWithSphere() {
        Sphere sphere1 = new Sphere(new Vector3f(0, 0, 0), 1.0f);
        Sphere sphere2 = new Sphere(new Vector3f(3, 3, 3), 1.0f);

        boolean result = Sphere.isSphereColliding(sphere1, sphere2);

        assertFalse(result, "Spheres should not be colliding");
    }
}