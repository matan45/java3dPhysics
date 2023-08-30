package collisionDetection.util;

import math.Vector3f;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Set;

import static org.junit.jupiter.api.Assertions.assertEquals;

class QuickHull3DTest {

    @Test
    void quickHullTest() {
        Set<Vector3f> points = new HashSet<>();
        points.add(new Vector3f(1.0f, 1.0f, 0.0f));
        points.add(new Vector3f(9.0f, 2.0f, 0.0f));
        points.add(new Vector3f(7.0f, 5.0f, 0.0f));
        points.add(new Vector3f(2.0f, 4.0f, 0.0f));
        points.add(new Vector3f(4.0f, 3.0f, 0.0f));
        points.add(new Vector3f(6.0f, 1.0f, 0.0f));


        Set<Vector3f> convexHull = QuickHull3D.findConvexHull(new ArrayList<>(points));
        assertEquals(points, convexHull);
    }

}