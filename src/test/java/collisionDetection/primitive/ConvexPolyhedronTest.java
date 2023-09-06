package collisionDetection.primitive;


import math.Vector3f;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class ConvexPolyhedronTest {

    @Test
    public void testIsPointInside() {
        // Create a ConvexPolyhedron instance for testing
        List<Vector3f> vertices = new ArrayList<>();
        vertices.add(new Vector3f(0, 0, 0));
        vertices.add(new Vector3f(5, 0, 0));
        vertices.add(new Vector3f(5, 5, 0));
        vertices.add(new Vector3f(0, 5, 0));

        ConvexPolyhedron polyhedron = new ConvexPolyhedron(vertices);

        // Test points
        Vector3f insidePoint = new Vector3f(2, 2, 0);
        Vector3f outsidePoint = new Vector3f(6, 6, 6);

        assertTrue(polyhedron.isPointInside(insidePoint));
        assertFalse(polyhedron.isPointInside(outsidePoint));
    }


    @Test
    public void testClosestPoint() {
        // Create a ConvexPolyhedron instance for testing
        List<Vector3f> vertices = new ArrayList<>();
        vertices.add(new Vector3f(0, 0, 0));
        vertices.add(new Vector3f(5, 0, 0));
        vertices.add(new Vector3f(5, 5, 0));
        vertices.add(new Vector3f(0, 5, 0));
        ConvexPolyhedron polyhedron = new ConvexPolyhedron(vertices);

        // Test point
        Vector3f testPoint = new Vector3f(2, 7, 2);

        Vector3f closestPoint = polyhedron.closestPoint(testPoint);
        // Calculate the expected closest point manually or using your own method
        Vector3f expectedClosestPoint = new Vector3f(0, 5, 0);

        assertEquals(expectedClosestPoint, closestPoint);
    }


}