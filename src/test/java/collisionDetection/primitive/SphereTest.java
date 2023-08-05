package collisionDetection.primitive;

import org.joml.Vector3f;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

class SphereTest {

    @Test
    void testSphereClosestPointPair() {
        Sphere sphere1 = new Sphere(new Vector3f(0, 0, 0), 1.0f);
        Sphere sphere2 = new Sphere(new Vector3f(1, 1, 1), 1.0f);

        CollisionResult expected = new CollisionResult(true, 1.7320507764816284, new Vector3f(0, 0, 0), new Vector3f(1, 1, 1));
        CollisionResult result = Sphere.isSphereColliding(sphere1, sphere2);

        assertEquals(expected, result, "Spheres should be colliding");
    }

    @Test
    void testSphereClosestPointPairNotCollide() {
        Sphere sphere1 = new Sphere(new Vector3f(0, 0, 0), 1.0f);
        Sphere sphere2 = new Sphere(new Vector3f(3, 3, 3), 1.0f);

        CollisionResult expected = new CollisionResult(false, 3.1961522102355957, new Vector3f(2.0773504f, 2.0773504f, 2.0773504f), new Vector3f(0, 0, 0));
        CollisionResult result = Sphere.isSphereColliding(sphere1, sphere2);

        assertEquals(expected, result);
    }
}