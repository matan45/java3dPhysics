package collisionDetection.primitive;


import math.Vector3f;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class CylinderTest {

    @Test
    public void testIsPointInside() {
        // Create a cylinder with a center at (0, 0, 0), radius 1, and height 2
        Cylinder cylinder = new Cylinder(new Vector3f(0, 0, 0), 1, 2);

        // Test points that are inside and outside the cylinder
        assertTrue(cylinder.isPointInside(new Vector3f(0, 0.5f, 0))); // Inside
        assertFalse(cylinder.isPointInside(new Vector3f(1.5f, 0.5f, 0))); // Outside
    }

    @Test
    public void testClosestPoint() {
        // Create a cylinder with a center at (0, 0, 0), radius 1, and height 2
        Cylinder cylinder = new Cylinder(new Vector3f(0, 0, 0), 1, 2);

        // Test points and their expected closest points on the cylinder
        Vector3f expectedClosest = new Vector3f(1, 2, 0);  // On the base

        Vector3f closest = cylinder.closestPoint(new Vector3f(2, 2, 0));

        assertEquals(expectedClosest, closest);
    }

    @Test
    public void testCollidingCylinders() {
        Cylinder cylinder1 = new Cylinder(new Vector3f(1.0f, 1.0f, 1.0f), 1.0f, 2.0f);
        Cylinder cylinder2 = new Cylinder(new Vector3f(2.0f, 2.0f, 2.0f), 1.0f, 2.0f);
        assertTrue(Cylinder.isCylinderColliding(cylinder1, cylinder2));
    }

    @Test
    public void testNonCollidingCylinders() {
        Cylinder cylinder1 = new Cylinder(new Vector3f(1.0f, 1.0f, 1.0f), 1.0f, 2.0f);
        Cylinder cylinder2 = new Cylinder(new Vector3f(5.0f, 5.0f, 5.0f), 1.0f, 2.0f);
        assertFalse(Cylinder.isCylinderColliding(cylinder1, cylinder2));
    }
}