package collisionDetection.primitive;


import org.joml.Vector3f;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class CylinderTest {
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