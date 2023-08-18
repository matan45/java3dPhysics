package collisionDetection.primitive;

import math.Vector3f;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class PlaneTest {
    @Test
    public void testCollidingPlanes() {
        Plane plane1 = new Plane(new Vector3f(1.0f, 0.0f, 0.0f), 0.0f);
        Plane plane2 = new Plane(new Vector3f(2.0f, 0.0f, 0.0f), 0.0f);
        assertTrue(Plane.isPlaneColliding(plane1, plane2));
    }

    @Test
    public void testNonCollidingPlanes() {
        Plane plane1 = new Plane(new Vector3f(1.0f, 0.0f, 0.0f), 0.0f);
        Plane plane2 = new Plane(new Vector3f(0.0f, 1.0f, 0.0f), 0.0f);
        assertFalse(Plane.isPlaneColliding(plane1, plane2));
    }
}