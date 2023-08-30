package com.boot;

import collisionDetection.util.hull.QuickHull3D;
import math.Vector3f;

import java.util.ArrayList;
import java.util.List;

public class Main {
    public static void main(String[] args) {
        List<Vector3f> points = new ArrayList<>();
        points.add(new Vector3f(1.0f, 2.0f, 3.0f));
        points.add(new Vector3f(1.0f, 2.0f, 3.0f));
        points.add(new Vector3f(4.0f, 5.0f, 6.0f));
        points.add(new Vector3f(7.0f, 8.0f, 9.0f));

        points.add(new Vector3f(3.0f, 1.0f, 3.0f));
        points.add(new Vector3f(7.0f, 2.0f, 2.0f));
        points.add(new Vector3f(9.0f, 6.0f, 8.0f));

        points.add(new Vector3f(9.0f, 10.0f, 7.0f));
        points.add(new Vector3f(3.0f, 8.0f, 12.0f));
        points.add(new Vector3f(15.0f, 21.0f, 4.0f));

        points.add(new Vector3f(3.0f, 1.0f, 3.0f));
        points.add(new Vector3f(7.0f, 2.0f, 2.0f));
        points.add(new Vector3f(9.0f, 6.0f, 8.0f));

        points.add(new Vector3f(1.0f, 2.0f, 3.0f));
        points.add(new Vector3f(1.0f, 2.0f, 3.0f));
        points.add(new Vector3f(4.0f, 5.0f, 6.0f));


        QuickHull3D quickHull = new QuickHull3D(points);
        List<Vector3f> convexHullVertices = quickHull.computeConvexHull();

        // Print the convex hull vertices
        for (Vector3f vertex : convexHullVertices) {
            System.out.println("Convex Hull Vertex: " + vertex.x + ", " + vertex.y + ", " + vertex.z);
        }
    }

}