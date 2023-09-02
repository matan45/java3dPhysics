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

    @Test
    public void testLineCollision() {
        // Test cases where the lines intersect
        Line line1 = new Line(new Vector3f(0, 0, 0), new Vector3f(1, 1, 0));
        Line line2 = new Line(new Vector3f(0, 1, 0), new Vector3f(1, 0, 0));
        assertTrue(Line.isLineColliding(line1, line2));

        Line line3 = new Line(new Vector3f(0, 0, 0), new Vector3f(1, 1, 0));
        Line line4 = new Line(new Vector3f(0.5f, 0.5f, 0), new Vector3f(1, 0, 0));
        assertTrue(Line.isLineColliding(line3, line4));

        // Test cases where the lines do not intersect
        Line line5 = new Line(new Vector3f(0, 0, 0), new Vector3f(1, 1, 0));
        Line line6 = new Line(new Vector3f(2, 2, 0), new Vector3f(3, 3, 0));
        assertFalse(Line.isLineColliding(line5, line6));

        Line line7 = new Line(new Vector3f(0, 0, 0), new Vector3f(1, 1, 0));
        Line line8 = new Line(new Vector3f(0, 2, 0), new Vector3f(1, 3, 0));
        assertFalse(Line.isLineColliding(line7, line8));

        // Test cases where the lines are parallel but not collinear
        Line line9 = new Line(new Vector3f(0, 0, 0), new Vector3f(1, 1, 0));
        Line line10 = new Line(new Vector3f(0, 0, 1), new Vector3f(1, 1, 1));
        assertFalse(Line.isLineColliding(line9, line10));

        Line line11 = new Line(new Vector3f(2, 2, 2), new Vector3f(4, 4, 4));
        Line line12 = new Line(new Vector3f(1, -1, 1), new Vector3f(1, 1, 1));
        assertFalse(Line.isLineColliding(line11, line12));
    }

}