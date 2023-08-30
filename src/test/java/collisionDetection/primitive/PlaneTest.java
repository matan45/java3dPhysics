package collisionDetection.primitive;

import math.Vector3f;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class PlaneTest {

    @Test
    void testIsPointInside() {
        Vector3f normal = new Vector3f(0.0f, 1.0f, 0.0f);
        float distance = 0.0f;
        Plane plane = new Plane(normal, distance);

        // Test a point on the plane
        Vector3f onPlanePoint = new Vector3f(1.0f, 0.0f, 1.0f);
        assertTrue(plane.isPointInside(onPlanePoint));

        // Test a point above the plane
        Vector3f abovePlanePoint = new Vector3f(1.0f, 2.0f, 1.0f);
        assertFalse(plane.isPointInside(abovePlanePoint));

        // Test a point below the plane
        Vector3f belowPlanePoint = new Vector3f(1.0f, -1.0f, 1.0f);
        assertFalse(plane.isPointInside(belowPlanePoint));
    }

    @Test
    void testClosestPoint() {
        Vector3f normal = new Vector3f(0.0f, 1.0f, 0.0f);
        float distance = 0.0f;
        Plane plane = new Plane(normal, distance);

        Vector3f point = new Vector3f(1.0f, 2.0f, 1.0f);
        Vector3f closest = plane.closestPoint(point);

        // The closest point should be on the plane
        assertEquals(new Vector3f(1, 0, 1),closest);
    }

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