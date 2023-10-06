package collisionDetection.util;

import math.Vector3f;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
//not working need to do from the libs
public class Quick {
    public static Set<Vector3f> findConvexHull(List<Vector3f> points) {
        if (points.size() < 4)
            return new HashSet<>(points);

        List<Vector3f> convexHull = new ArrayList<>();

        Vector3f A = points.get(0);
        Vector3f B = points.get(1);
        Vector3f C = points.get(2);

        convexHull.add(A);
        convexHull.add(B);
        convexHull.add(C);

        List<Vector3f> S1 = new ArrayList<>();
        List<Vector3f> S2 = new ArrayList<>();

        for (Vector3f point : points) {
            if (!point.equals(A) && !point.equals(B) && !point.equals(C)) {
                if (isOnPositiveSide(point, A, B, C)) {
                    S1.add(point);
                } else {
                    S2.add(point);
                }
            }
        }

        findHull3D(convexHull, S1, A, B, C);
        findHull3D(convexHull, S2, B, A, C);

        // Remove duplicates from the convexHull list
        return new HashSet<>(convexHull);
    }

    private static void findHull3D(List<Vector3f> convexHull, List<Vector3f> Sk, Vector3f A, Vector3f B, Vector3f C) {
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

    private static boolean isOnPositiveSide(Vector3f p, Vector3f a, Vector3f b, Vector3f c) {
        // Calculate the normal vector of the plane
        Vector3f normal = calculateNormal(a, b, c);

        // Calculate the vector from point 'a' to 'p'
        Vector3f ap = p.sub(a);

        // Calculate the dot product of the normal vector and the vector 'ap'
        double dotProduct = normal.dot(ap);

        // If the dot product is positive or zero, the point is on the positive side of the plane
        return dotProduct >= 0;
    }

    private static Vector3f calculateNormal(Vector3f a, Vector3f b, Vector3f c) {
        Vector3f vectorAB = b.sub(a);
        Vector3f vectorAC = c.sub(a);

        // Calculate the cross product of vectors AB and AC to get the normal vector
        Vector3f normal = vectorAB.cross(vectorAC);

        return normal.normalize();
    }

    private static Vector3f findFarthestPoint(List<Vector3f> points, Vector3f a, Vector3f b, Vector3f c) {
        double maxDistance = -1;
        Vector3f farthestPoint = null;

        for (Vector3f point : points) {
            if (!point.equals(a) && !point.equals(b) && !point.equals(c)) {
                double distance = distanceToPlane(point, a, b, c);
                if (distance > maxDistance) {
                    maxDistance = distance;
                    farthestPoint = point;
                }
            }
        }

        return farthestPoint;
    }

    private static double distanceToPlane(Vector3f p, Vector3f a, Vector3f b, Vector3f c) {
        Vector3f normal = calculateNormal(a, b, c);
        Vector3f ap = p.sub(a);
        return Math.abs(normal.dot(ap));
    }

}
