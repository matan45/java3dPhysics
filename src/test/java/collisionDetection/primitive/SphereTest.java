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


}