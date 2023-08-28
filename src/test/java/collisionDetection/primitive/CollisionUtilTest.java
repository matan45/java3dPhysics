package collisionDetection.primitive;

import math.Vector3f;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class CollisionUtilTest {

    @Test
    void testConvexHull3D_JM() {
        List<Vector3f> vertices = new ArrayList<>();
        vertices.add(new Vector3f(0, 3, 0));
        vertices.add(new Vector3f(2, 3, 0));
        vertices.add(new Vector3f(1, 1, 0));
        vertices.add(new Vector3f(2, 1, 0));
        vertices.add(new Vector3f(3, 0, 0));
        vertices.add(new Vector3f(0, 0, 0));
        vertices.add(new Vector3f(3, 3, 0));

        List<Vector3f> convexHull = CollisionUtil.convexHull3D_JM(vertices);

        List<Vector3f> expectedHull  = new ArrayList<>();
        expectedHull .add(new Vector3f(0, 3, 0));
        expectedHull .add(new Vector3f(0, 0, 0));
        expectedHull .add(new Vector3f(3, 0, 0));
        expectedHull .add(new Vector3f(3, 3, 1));

        assertEquals(expectedHull.size(), convexHull.size());
        for (int i = 0; i < expectedHull.size(); i++) {
            Vector3f expected = expectedHull.get(i);
            Vector3f actual = convexHull.get(i);
            assertEquals(expected.x, actual.x, 1e-6); // Tolerance for float comparison
            assertEquals(expected.y, actual.y, 1e-6);
            assertEquals(expected.z, actual.z, 1e-6);
        }
    }

    @Test
    void testConvexHull3D_GS() {
        List<Vector3f> vertices = new ArrayList<>();
        vertices.add(new Vector3f(0, 3, 0));
        vertices.add(new Vector3f(2, 3, 0));
        vertices.add(new Vector3f(1, 1, 0));
        vertices.add(new Vector3f(2, 1, 0));
        vertices.add(new Vector3f(3, 0, 0));
        vertices.add(new Vector3f(0, 0, 0));
        vertices.add(new Vector3f(3, 3, 0));

        List<Vector3f> convexHull = CollisionUtil.convexHull3D_GS(vertices);

        List<Vector3f> expectedHull  = new ArrayList<>();
        expectedHull .add(new Vector3f(0, 3, 0));
        expectedHull .add(new Vector3f(0, 0, 0));
        expectedHull .add(new Vector3f(3, 0, 0));
        expectedHull .add(new Vector3f(3, 3, 1));

        assertEquals(expectedHull.size(), convexHull.size());
        for (int i = 0; i < expectedHull.size(); i++) {
            Vector3f expected = expectedHull.get(i);
            Vector3f actual = convexHull.get(i);
            assertEquals(expected.x, actual.x, 1e-6);
            assertEquals(expected.y, actual.y, 1e-6);
            assertEquals(expected.z, actual.z, 1e-6);
        }
    }
    @Test
    void testConvexHull3D_AMC() {
        List<Vector3f> vertices = new ArrayList<>();
        vertices.add(new Vector3f(0, 3, 0));
        vertices.add(new Vector3f(2, 3, 0));
        vertices.add(new Vector3f(1, 1, 0));
        vertices.add(new Vector3f(2, 1, 0));
        vertices.add(new Vector3f(3, 0, 0));
        vertices.add(new Vector3f(0, 0, 0));
        vertices.add(new Vector3f(3, 3, 0));

        List<Vector3f> convexHull = CollisionUtil.convexHull3D_AMC(vertices);

        List<Vector3f> expectedHull  = new ArrayList<>();
        expectedHull .add(new Vector3f(0, 3, 0));
        expectedHull .add(new Vector3f(0, 0, 0));
        expectedHull .add(new Vector3f(3, 0, 0));
        expectedHull .add(new Vector3f(3, 3, 1));

        assertEquals(expectedHull.size(), convexHull.size());
        for (int i = 0; i < expectedHull.size(); i++) {
            Vector3f expected = expectedHull.get(i);
            Vector3f actual = convexHull.get(i);
            assertEquals(expected.x, actual.x, 1e-6); // Tolerance for float comparison
            assertEquals(expected.y, actual.y, 1e-6);
            assertEquals(expected.z, actual.z, 1e-6);
        }
    }

    @Test
    void testConvexHull3D_GFG() {
        List<Vector3f> vertices = new ArrayList<>();
        vertices.add(new Vector3f(0, 3, 0));
        vertices.add(new Vector3f(2, 3, 0));
        vertices.add(new Vector3f(1, 1, 0));
        vertices.add(new Vector3f(2, 1, 0));
        vertices.add(new Vector3f(3, 0, 0));
        vertices.add(new Vector3f(0, 0, 0));
        vertices.add(new Vector3f(3, 3, 0));

        List<Vector3f> convexHull = CollisionUtil.convexHull3D_GFG(vertices);

        // You need to define the expected convex hull here based on your input vertices.
        List<Vector3f> expectedHull  = new ArrayList<>();
        expectedHull .add(new Vector3f(0, 3, 0));
        expectedHull .add(new Vector3f(0, 0, 0));
        expectedHull .add(new Vector3f(3, 0, 0));
        expectedHull .add(new Vector3f(3, 3, 1));

        // Compare the expected convex hull with the calculated one
        assertEquals(expectedHull , convexHull);
    }
}