package com.boot;

import math.Vector3f;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class QuickHull3D {
    public static Set<Vector3f> findConvexHull(List<Vector3f> points) {
        if (points.size() < 3)
            return new HashSet<>(points);

        List<Vector3f> convexHull = new ArrayList<>();

        Vector3f A = points.get(0); // Assuming points list is not empty
        Vector3f B = points.get(1); // Assuming points list has at least 2 points
        Vector3f C = points.get(2); // Assuming points list has at least 3 points

        convexHull.add(A);
        convexHull.add(B);
        convexHull.add(C);

        List<Vector3f> S1 = new ArrayList<>();
        List<Vector3f> S2 = new ArrayList<>();

        for (Vector3f point : points) {
            if (!point.equals(A) &&!point.equals(B) && !point.equals(C)) {
                if (isOnPositiveSide(point, A, B, C)) {
                    S1.add(point);
                } else {
                    S2.add(point);
                }
            }
        }

        findHull3D(convexHull, S1, A, B, C);
        findHull3D(convexHull, S2, B, A, C);

        return new HashSet<>(convexHull);
    }

    public static void findHull3D(List<Vector3f> convexHull, List<Vector3f> Sk, Vector3f A, Vector3f B, Vector3f C) {
        if (Sk.isEmpty()) {
            return;
        }

        Vector3f E = findFarthestPoint(Sk, A, B, C);

        convexHull.add(E);

        List<Vector3f> S1 = new ArrayList<>();
        List<Vector3f> S2 = new ArrayList<>();
        List<Vector3f> S3 = new ArrayList<>();
        List<Vector3f> S4 = new ArrayList<>();

        for (Vector3f point : Sk) {
            if (point != E) {
                if (isOnPositiveSide(point, B, C, E)) {
                    S1.add(point);
                } else if (isOnPositiveSide(point, B, A, E)) {
                    S2.add(point);
                } else if (isOnPositiveSide(point, C, B, E)) {
                    S3.add(point);
                } else {
                    S4.add(point);
                }
            }
        }

        findHull3D(convexHull, S1, A, E, B);
        findHull3D(convexHull, S2, B, E, C);
        findHull3D(convexHull, S3, C, E, A);
        findHull3D(convexHull, S4, E, B, C);
    }

    public static boolean isOnPositiveSide(Vector3f p, Vector3f a, Vector3f b, Vector3f c) {
        // Calculate the normal vector of the plane
        Vector3f normal = calculateNormal(a, b, c);

        // Calculate the vector from point 'a' to 'p'
        Vector3f ap = new Vector3f(p.x - a.x, p.y - a.y, p.z - a.z);

        // Calculate the dot product of the normal vector and the vector 'ap'
        double dotProduct = normal.dot(ap);

        // If the dot product is positive or zero, the point is on the positive side of the plane
        return dotProduct >= 0;
    }

    public static Vector3f calculateNormal(Vector3f a, Vector3f b, Vector3f c) {
        Vector3f vectorAB = new Vector3f(b.x - a.x, b.y - a.y, b.z - a.z);
        Vector3f vectorAC = new Vector3f(c.x - a.x, c.y - a.y, c.z - a.z);

        // Calculate the cross product of vectors AB and AC to get the normal vector
        Vector3f normal = vectorAB.cross(vectorAC);

        return normal.normalize();
    }

    public static Vector3f findFarthestPoint(List<Vector3f> points, Vector3f a, Vector3f b, Vector3f c) {
        double maxDistance = -1;
        Vector3f farthestPoint = null;

        for (Vector3f point : points) {
            if (point != a && point != b && point != c) {
                double distance = distanceToPlane(point, a, b, c);
                if (distance > maxDistance) {
                    maxDistance = distance;
                    farthestPoint = point;
                }
            }
        }

        return farthestPoint;
    }

    public static double distanceToPlane(Vector3f p, Vector3f a, Vector3f b, Vector3f c) {
        Vector3f normal = calculateNormal(a, b, c);
        Vector3f ap = new Vector3f(p.x - a.x, p.y - a.y, p.z - a.z);
        return Math.abs(normal.dot(ap));
    }

    public static void main(String[] args) {
        List<Vector3f> points = new ArrayList<>();
        points.add(new Vector3f(1.0f, 1.0f, 0.0f));
        points.add(new Vector3f(2.0f, 4.0f, 0.0f));
        points.add(new Vector3f(4.0f, 3.0f, 0.0f));
        points.add(new Vector3f(6.0f, 1.0f, 0.0f));
        points.add(new Vector3f(7.0f, 5.0f, 0.0f));
        points.add(new Vector3f(9.0f, 2.0f, 0.0f));


        Set<Vector3f> convexHull = findConvexHull(points);
        System.out.println("Convex Hull:");
        for (Vector3f point : convexHull) {
            System.out.println("(" + point.x + ", " + point.y + ", " + point.z + ")");
        }
    }

}
