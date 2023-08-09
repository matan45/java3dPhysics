package collisionDetection.primitive;

import org.joml.Vector3f;

import java.util.Arrays;

public class Triangle {
    private Vector3f[] vertices; // Vertices of the triangle

    public Triangle(Vector3f vertex1, Vector3f vertex2, Vector3f vertex3) {
        vertices = new Vector3f[] { vertex1, vertex2, vertex3 };
    }

    public void setVertices(Vector3f[] vertices) {
        this.vertices = vertices;
    }

    public Vector3f[] getVertices() {
        return vertices;
    }

    public static boolean isTriangleColliding(Triangle triangle1, Triangle triangle2) {
        Vector3f[] vertices1 = triangle1.getVertices();
        Vector3f[] vertices2 = triangle2.getVertices();

        // Check for overlap along all three axes of triangles
        for (int i = 0; i < 3; i++) {
            Vector3f axis1 = vertices1[(i + 1) % 3].sub(vertices1[i]).cross(vertices1[(i + 2) % 3].sub(vertices1[i]));
            if (isSeparatingAxis(axis1, vertices1, vertices2)) {
                return false;
            }

            Vector3f axis2 = vertices2[(i + 1) % 3].sub(vertices2[i]).cross(vertices2[(i + 2) % 3].sub(vertices2[i]));
            if (isSeparatingAxis(axis2, vertices1, vertices2)) {
                return false;
            }
        }

        // Check for overlap along the normal of triangle1
        Vector3f normal1 = vertices1[1].sub(vertices1[0]).cross(vertices1[2].sub(vertices1[0]));
        if (isSeparatingAxis(normal1, vertices1, vertices2)) {
            return false;
        }

        // Check for overlap along the normal of triangle2
        Vector3f normal2 = vertices2[1].sub(vertices2[0]).cross(vertices2[2].sub(vertices2[0]));
        return !isSeparatingAxis(normal2, vertices1, vertices2);
    }

    public static boolean isSeparatingAxis(Vector3f axis, Vector3f[] vertices1, Vector3f[] vertices2) {
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

        return (max1 < min2) || (max2 < min1);
    }


    @Override
    public String toString() {
        return "Triangle{" +
                "vertices=" + Arrays.toString(vertices) +
                '}';
    }
}
