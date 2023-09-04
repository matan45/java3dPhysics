package collisionDetection.primitive;

import collisionDetection.primitive.terrain.TerrainShape;
import math.Maths;
import math.Vector3f;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class TerrainShapeTest {

    private TerrainShape terrain;

    @BeforeEach
    public void setUp() {
        float[][] heightData = new float[10][10];
        for (int i = 0; i < heightData.length; i++) {
            for (int j = 0; j < heightData[i].length; j++) {
                heightData[i][j] = Maths.getRandomNumber(0, 5);
            }
        }

        // Define a simple AABB for terrain borders
        AABB borders = new AABB(new Vector3f(-1, -1, -1), new Vector3f(12, 12, 12));

        // Initialize the TerrainShape object
        terrain = new TerrainShape(heightData, borders, new Vector3f(1,1,1));
    }

    @Test
    public void testIsPointOnGround() {
        // Create a test point
        Vector3f testPoint = new Vector3f(0, 0, 0);

        // Check if the point is on the ground
        boolean onGround = terrain.isPointOnGround(testPoint);

        assertTrue(onGround, "Point should be on the ground");
    }

    @Test
    void testIsNotPointOnGround() {
        // Create a test point
        Vector3f testPoint = new Vector3f(5, 5, 1);

        // Check if the point is on the ground
        boolean onGround = terrain.isPointOnGround(testPoint);

        assertFalse(onGround, "Point should not be on the ground");
    }

    @Test
    public void testRayTerrainNotIntersection() {
        // Create a test ray
        Ray testRay = new Ray(new Vector3f(0, 10, 0), new Vector3f(0, 1, 0));

        // Perform the ray-terrain intersection
        Vector3f intersection = terrain.rayTerrainIntersection(testRay);

        assertNull(intersection, "Ray should not intersect the terrain");
    }

    @Test
    public void testRayTerrainIntersection() {
        // Create a test ray
        Ray testRay = new Ray(new Vector3f(0, 5, 0), new Vector3f(0, -1, 0));

        // Perform the ray-terrain intersection
        Vector3f intersection = terrain.rayTerrainIntersection(testRay);

        assertNotNull(intersection, "Ray should intersect the terrain");
    }
}