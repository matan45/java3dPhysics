package collisionDetection.primitive;

import org.joml.Vector3f;

import java.util.Arrays;

public class Triangle {
    private Vector3f[] vertices; // Vertices of the triangle

    public Triangle(Vector3f vertex1, Vector3f vertex2, Vector3f vertex3) {
        vertices = new Vector3f[]{vertex1, vertex2, vertex3};
    }

    public void setVertices(Vector3f[] vertices) {
        this.vertices = vertices;
    }

    public Vector3f[] getVertices() {
        return vertices;
    }

    private Plane FromTriangle() {
        Plane result = new Plane(new Vector3f(), 0);
        result.setNormal(vertices[1].sub(vertices[0]).cross(vertices[2].sub(vertices[0])));
        result.setDistance(result.getNormal().dot(vertices[0]));
        return result;
    }

    private boolean pointInTriangleNormals(Vector3f point) {
        Vector3f a = vertices[0].sub(point);
        Vector3f b = vertices[1].sub(point);
        Vector3f c = vertices[2].sub(point);

        Vector3f normPBC = b.cross(c); // Normal of PBC (u)
        Vector3f normPCA = c.cross(a); // Normal of PCA (v)
        Vector3f normPAB = a.cross(b); // Normal of PAB (w)

        if (normPBC.dot(normPCA) < 0.0f) {
            return false;
        } else if (normPBC.dot(normPAB) < 0.0f) {
            return false;
        }

        return true;
    }

    public Vector3f closestPoint(Vector3f point) {
        Plane plane = FromTriangle();
        Vector3f closest = plane.closestPoint(point);

        if (pointInTriangleNormals(closest)) {
            return closest;
        }

        Vector3f c1 = ClosestPoint(vertices[0], vertices[1], point); // Line AB
        Vector3f c2 = ClosestPoint(vertices[1], vertices[2], point); // Line AB
        Vector3f c3 = ClosestPoint(vertices[2], vertices[0], point); // Line AB

        Vector3f subsq1=point.sub(c1);
        Vector3f subsq2=point.sub(c2);
        Vector3f subsq3=point.sub(c3);

        float magSq1 = subsq1.dot(subsq1);
        float magSq2 = subsq2.dot(subsq2);
        float magSq3 = subsq3.dot(subsq3);

        if (magSq1 < magSq2 && magSq1 < magSq3) {
            return c1;
        } else if (magSq2 < magSq1 && magSq2 < magSq3) {
            return c2;
        }
        return c3;
    }

    private Vector3f ClosestPoint(Vector3f start, Vector3f end, Vector3f point) {
        Vector3f lVec = end.sub(start); // Line Vector
        float t = point.sub(start).dot(lVec) / lVec.dot(lVec);
        t = Math.max(t, 0.0f); // Clamp to 0
        t = Math.min(t, 1.0f); // Clamp to 1
        return start.add(lVec.mul(t));
    }


    public static boolean isTriangleColliding(Triangle triangle1, Triangle triangle2) {
        for (int i = 0; i < 3; i++) {
            Vector3f edge1Start = triangle1.getVertices()[i];
            Vector3f edge1End = triangle1.getVertices()[(i + 1) % 3];

            for (int j = 0; j < 3; j++) {
                Vector3f edge2Start = triangle2.getVertices()[j];
                Vector3f edge2End = triangle2.getVertices()[(j + 1) % 3];

                Vector3f axis = edge1End.sub(edge1Start).cross(edge2End.sub(edge2Start));

                if (!isSeparatingAxis(axis, triangle1, triangle2)) {
                    return false;
                }
            }
        }
        return true;
    }

    public static boolean isSeparatingAxis(Vector3f axis, Triangle triangle1, Triangle triangle2) {
        // Project triangles onto the axis and check for separation
        float projection1 = projectOntoAxis(axis, triangle1);
        float projection2 = projectOntoAxis(axis, triangle2);

        // Check for separation
        return projection1 > projection2 || projection2 > projection1;
    }

    public static float projectOntoAxis(Vector3f axis, Triangle triangle) {
        float min = Float.POSITIVE_INFINITY;
        float max = Float.NEGATIVE_INFINITY;

        for (int i = 0; i < 3; i++) {
            float projection = axis.dot(triangle.getVertices()[i]);
            min = Math.min(min, projection);
            max = Math.max(max, projection);
        }

        return min;
    }


    @Override
    public String toString() {
        return "Triangle{" +
                "vertices=" + Arrays.toString(vertices) +
                '}';
    }
}
