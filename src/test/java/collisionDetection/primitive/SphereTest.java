package collisionDetection.primitive;

import math.Vector3f;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class SphereTest {

    @Test
    public void testIsPointInside() {
        Sphere sphere = new Sphere(new Vector3f(0, 0, 0), 5);

        assertTrue(sphere.isPointInside(new Vector3f(3, 0, 0)));
        assertFalse(sphere.isPointInside(new Vector3f(7, 0, 0)));
    }

    @Test
    public void testClosestPoint() {
        Sphere sphere = new Sphere(new Vector3f(0, 0, 0), 5);

        Vector3f closest = sphere.closestPoint(new Vector3f(7, 0, 0));
        assertEquals(new Vector3f(5,0,0), closest); // Check that the length is approximately 5
    }

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