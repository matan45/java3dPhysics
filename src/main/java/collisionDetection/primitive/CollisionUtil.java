package collisionDetection.primitive;

import math.Vector3f;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class CollisionUtil {
    //Jarvis March (Gift Wrapping) Algorithm
    public static List<Vector3f> convexHull3D_JM(List<Vector3f> vertices) {
        if (vertices.size() < 4) {
            // Convex hull is not possible with fewer than 4 points
            return vertices;
        }

        List<Vector3f> convexHull = new ArrayList<>();

        // Find the point with the lowest x-coordinate
        Vector3f startPoint = findLowestXVertex(vertices);

        convexHull.add(startPoint);
        Vector3f currentPoint = startPoint;

        do {
            Vector3f nextPoint = null;

            for (Vector3f vertex : vertices) {
                if (vertex != currentPoint) {
                    if (nextPoint == null || isLeftTurn(currentPoint, nextPoint, vertex)) {
                        nextPoint = vertex;
                    }
                }
            }

            assert nextPoint != null;
            if (!areEqual(startPoint, nextPoint)) {
                convexHull.add(nextPoint);
                currentPoint = nextPoint;
            }
            System.out.println(startPoint);
            System.out.println(currentPoint);
        } while (!areEqual(startPoint, currentPoint));

        return convexHull;
    }

    // Helper function to find the point with the lowest x-coordinate
    private static Vector3f findLowestXVertex(List<Vector3f> vertices) {
        Vector3f lowest = vertices.get(0);
        for (Vector3f vertex : vertices) {
            if (vertex.x < lowest.x) {
                lowest = vertex;
            }
        }
        return lowest;
    }

    private static boolean areEqual(Vector3f a, Vector3f b) {
        float tolerance = 1e-6f; // Define a suitable tolerance
        return Math.abs(a.x - b.x) < tolerance &&
                Math.abs(a.y - b.y) < tolerance &&
                Math.abs(a.z - b.z) < tolerance;
    }

    // Helper function to determine if a point is to the left of a line segment
    private static boolean isLeftTurn(Vector3f a, Vector3f b, Vector3f c) {
        float crossProduct = (b.x - a.x) * (c.y - a.y) * (b.z - a.z) - (c.x - a.x) * (b.y - a.y) * (b.z - a.z);
        return crossProduct > 0;
    }

    //Graham's Scan Algorithm
    public static List<Vector3f> convexHull3D_GS(List<Vector3f> vertices) {
        if (vertices.size() < 4) {
            // Convex hull is not possible with fewer than 4 points
            return vertices;
        }

        List<Vector3f> sortedVertices = new ArrayList<>(vertices);
        sortVerticesByPolarAngle(sortedVertices);

        List<Vector3f> convexHull = new ArrayList<>();
        convexHull.add(sortedVertices.get(0));
        convexHull.add(sortedVertices.get(1));

        for (int i = 2; i < sortedVertices.size(); i++) {
            while (convexHull.size() >= 2 && !isLeftTurn(convexHull.get(convexHull.size() - 2), convexHull.get(convexHull.size() - 1), sortedVertices.get(i))) {
                convexHull.remove(convexHull.size() - 1);
            }
            convexHull.add(sortedVertices.get(i));
        }

        return convexHull;
    }

    private static void sortVerticesByPolarAngle(List<Vector3f> vertices) {
        final Vector3f pivot = findLowestXVertex(vertices);

        vertices.sort((a, b) -> {
            if (a == pivot) return -1;
            if (b == pivot) return 1;

            boolean aIsLeft = isLeftTurn(pivot, a, b);
            boolean bIsLeft = isLeftTurn(pivot, b, a);

            if (aIsLeft && !bIsLeft) return -1;
            if (!aIsLeft && bIsLeft) return 1;

            float angleA = angleBetweenVectors(pivot, a);
            float angleB = angleBetweenVectors(pivot, b);

            return Float.compare(angleA, angleB);
        });
    }

    private static float angleBetweenVectors(Vector3f a, Vector3f b) {
        // Calculate the dot product of a and b
        float dotProduct = a.x * b.x + a.y * b.y + a.z * b.z;

        // Calculate the magnitudes of the vectors
        float magnitudeA = (float) Math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
        float magnitudeB = (float) Math.sqrt(b.x * b.x + b.y * b.y + b.z * b.z);

        // Calculate the cosine of the angle between the vectors
        float cosAngle = dotProduct / (magnitudeA * magnitudeB);

        // Calculate and return the angle in radians
        return (float) Math.acos(cosAngle);
    }

    //Andrew's Monotone Chain Algorithm
    public static List<Vector3f> convexHull3D_AMC(List<Vector3f> vertices) {
        if (vertices.size() < 4) {
            // Convex hull is not possible with fewer than 4 points
            return vertices;
        }

        List<Vector3f> sortedVertices = new ArrayList<>(vertices);
        sortVerticesLexicographically(sortedVertices);

        List<Vector3f> upperHull = buildMonotoneChain(sortedVertices);
        Collections.reverse(sortedVertices);
        List<Vector3f> lowerHull = buildMonotoneChain(sortedVertices);

        // Combine the upper and lower hulls to get the final convex hull
        List<Vector3f> convexHull = new ArrayList<>(upperHull);
        convexHull.addAll(lowerHull);
        System.out.println(Arrays.toString(convexHull.toArray()));
        return convexHull;
    }

    private static void sortVerticesLexicographically(List<Vector3f> vertices) {
        // Sort the vertices lexicographically (first by x, then by y, then by z)
        vertices.sort((a, b) -> {
            if (a.x != b.x) return Float.compare(a.x, b.x);
            if (a.y != b.y) return Float.compare(a.y, b.y);
            return Float.compare(a.z, b.z);
        });
    }

    private static List<Vector3f> buildMonotoneChain(List<Vector3f> sortedVertices) {
        List<Vector3f> hull = new ArrayList<>();

        for (Vector3f vertex : sortedVertices) {
            while (hull.size() >= 2 && !isLeftTurn(hull.get(hull.size() - 2), hull.get(hull.size() - 1), vertex)) {
                hull.remove(hull.size() - 1);
            }
            if (!hull.contains(vertex)) {
                hull.add(vertex);
            }
        }
        System.out.println("hull "+Arrays.toString(hull.toArray()));
        return hull;
    }

    private static int orientation(Vector3f p, Vector3f q, Vector3f r) {
        float val = (q.y - p.y) * (r.z - q.z) -
                (q.z - p.z) * (r.y - q.y) +
                (q.x - p.x) * (r.y - q.y);

        if (val == 0) return 0;  // collinear
        return (val > 0) ? 1 : 2; // clock or counterclockwise
    }

    public static List<Vector3f> convexHull3D_GFG(List<Vector3f> vertices) {
        int n = vertices.size();
        if (n < 4) return vertices;

        List<Vector3f> hull = new ArrayList<>();

        int l = 0;
        for (int i = 1; i < n; i++)
            if (vertices.get(i).x < vertices.get(l).x)
                l = i;

        int p = l, q;
        do {
            hull.add(vertices.get(p));
            q = (p + 1) % n;
            for (int i = 0; i < n; i++) {
                if (orientation(vertices.get(p), vertices.get(i), vertices.get(q)) == 2)
                    q = i;
            }
            p = q;
        } while (p != l);

        return hull;
    }
}
