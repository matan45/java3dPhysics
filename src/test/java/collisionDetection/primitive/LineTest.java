package collisionDetection.primitive;

import math.Vector3f;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class LineTest {
    @Test
    public void testIsPointInside() {
        Vector3f start = new Vector3f(0, 0, 0);
        Vector3f end = new Vector3f(3, 0, 0);
        Line line = new Line(start, end);

        Vector3f pointInside = new Vector3f(1, 0, 0);
        Vector3f pointOutside = new Vector3f(4, 0, 0);

        assertTrue(line.isPointInside(pointInside));
        assertFalse(line.isPointInside(pointOutside));
    }

    @Test
    public void testClosestPoint() {
        Vector3f start = new Vector3f(0, 0, 0);
        Vector3f end = new Vector3f(0, 3, 0);
        Line line = new Line(start, end);

        Vector3f point = new Vector3f(1, 1, 1);
        Vector3f closest = line.closestPoint(point);

        assertEquals(new Vector3f(0, 1, 0), closest);
    }
}