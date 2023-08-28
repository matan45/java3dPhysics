package collisionDetection.primitive;

import math.Vector3f;

import java.util.List;

public class ConvexPolyhedron {
    private List<Vector3f> vertices;

    public ConvexPolyhedron(List<Vector3f> vertices) {
        this.vertices = vertices;
    }

    public List<Vector3f> getVertices() {
        return vertices;
    }

    public void setVertices(List<Vector3f> vertices) {
        this.vertices = vertices;
    }

    public static boolean isPolyhedronColliding(ConvexPolyhedron convexPolyhedron1, ConvexPolyhedron convexPolyhedron2) {
        List<Vector3f> vertices1 = convexPolyhedron1.getVertices();
        List<Vector3f> vertices2 = convexPolyhedron2.getVertices();

        // Loop through all the edges of both polyhedra
        for (int i = 0; i < vertices1.size(); i++) {
            Vector3f edgeStart = vertices1.get(i);
            Vector3f edgeEnd = vertices1.get((i + 1) % vertices1.size());

            Vector3f axis = edgeEnd.sub(edgeStart).normalize();

            if (isAxisSeparating(axis, vertices1, vertices2)) {
                return false;
            }
        }

        for (int i = 0; i < vertices2.size(); i++) {
            Vector3f edgeStart = vertices2.get(i);
            Vector3f edgeEnd = vertices2.get((i + 1) % vertices2.size());

            Vector3f axis = edgeEnd.sub(edgeStart).normalize();

            if (isAxisSeparating(axis, vertices1, vertices2)) {
                return false;
            }
        }

        return true;
    }

    private static boolean isAxisSeparating(Vector3f axis, List<Vector3f> vertices1, List<Vector3f> vertices2) {
        float min1 = Float.MAX_VALUE;
        float max1 = Float.MIN_VALUE;
        float min2 = Float.MAX_VALUE;
        float max2 = Float.MIN_VALUE;

        for (Vector3f vertex : vertices1) {
            float projection = vertex.dot(axis);
            min1 = Math.min(min1, projection);
            max1 = Math.max(max1, projection);
        }

        for (Vector3f vertex : vertices2) {
            float projection = vertex.dot(axis);
            min2 = Math.min(min2, projection);
            max2 = Math.max(max2, projection);
        }

        // Check for overlap along the axis
        // The axis is separating
        return max1 < min2 || min1 > max2;
    }


    @Override
    public String toString() {
        return "ConvexPolyhedron{" +
                "vertices=" + vertices +
                '}';
    }
}
