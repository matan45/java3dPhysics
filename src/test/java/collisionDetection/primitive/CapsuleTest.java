package collisionDetection.primitive;

import org.joml.Vector3f;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class CapsuleTest {

    @Test
    public void testIsCapsuleNotColliding() {
        Capsule capsule1 = new Capsule(new Vector3f(0, 0, 0), new Vector3f(0, 3, 0), 1.0f);
        Capsule capsule2 = new Capsule(new Vector3f(2, 2, 0), new Vector3f(2, 5, 0), 1.0f);

        CollisionResult expected = new CollisionResult(false, 0.2360679805278778, new Vector3f(0, 0, 0), new Vector3f(0, 0, 0));
        CollisionResult result = Capsule.isCapsuleColliding(capsule1, capsule2);

        assertEquals(expected, result);
    }

    @Test
    public void testIsCapsuleColliding() {
        Capsule capsule3 = new Capsule(new Vector3f(0, 0, 0), new Vector3f(0, 3, 0), 1.0f);
        Capsule capsule4 = new Capsule(new Vector3f(0, 2, 0), new Vector3f(0, 5, 0), 1.0f);

        CollisionResult expected = new CollisionResult(true, -1.0f, new Vector3f(0, 0, 0), new Vector3f(0, 0, 0));
        CollisionResult result = Capsule.isCapsuleColliding(capsule3, capsule4);
        // Assert that the capsules are not colliding
        assertEquals(expected, result);
    }
}