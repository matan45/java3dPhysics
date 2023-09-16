package collisionDetection.primitive;

import collisionDetection.narrowPhase.Shape;
import collisionDetection.narrowPhase.gjk.GJKSupport;
import collisionDetection.narrowPhase.sat.Interval;
import collisionDetection.narrowPhase.sat.SATSupport;
import math.Quaternion;
import math.Vector3f;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;

public class OBB implements Shape, SATSupport, GJKSupport {

    private Vector3f center;
    private Vector3f[] axis;
    private Vector3f halfExtents;

    public OBB(Vector3f center, Vector3f halfExtents) {
        this.center = center;
        this.axis = new Vector3f[]{Vector3f.XAxis, Vector3f.YAxis, Vector3f.ZAxis};
        this.halfExtents = halfExtents;
    }

    public Vector3f getCenter() {
        return center;
    }

    public void setCenter(Vector3f center) {
        this.center = center;
    }

    @Override
    public List<Vector3f> getAxis() {
        return List.of(axis);
    }

    @Override
    public List<Vector3f> getVertices() {
        List<Vector3f> vertices = new ArrayList<>(8);
        // Generate combinations of -1 and 1 for each axis
        int[] signs = new int[]{-1, 1};

        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) {
                for (int k = 0; k < 2; k++) {
                    // Calculate the vertex position using combinations of signs
                    Vector3f vertex = new Vector3f(
                            center.x + signs[i] * axis[0].x * halfExtents.x
                                    + signs[j] * axis[1].x * halfExtents.y
                                    + signs[k] * axis[2].x * halfExtents.z,
                            center.y + signs[i] * axis[0].y * halfExtents.x
                                    + signs[j] * axis[1].y * halfExtents.y
                                    + signs[k] * axis[2].y * halfExtents.z,
                            center.z + signs[i] * axis[0].z * halfExtents.x
                                    + signs[j] * axis[1].z * halfExtents.y
                                    + signs[k] * axis[2].z * halfExtents.z
                    );

                    vertices.add(vertex);
                }
            }
        }

        return vertices;
    }

    public void setAxis(Vector3f[] axis) {
        this.axis = axis;
    }

    public Vector3f getHalfExtents() {
        return halfExtents;
    }

    public void setHalfExtents(Vector3f halfExtents) {
        this.halfExtents = halfExtents;
    }

    public Vector3f getEdge(int edgeIndex) {
        // Calculate the endpoints of the edge
        Vector3f startPoint = new Vector3f(center);
        startPoint.add(axis[edgeIndex]);

        Vector3f endPoint = new Vector3f(center);
        endPoint.add(axis[edgeIndex].mul(halfExtents.get(edgeIndex)));

        // Calculate the edge vector
        return endPoint.sub(startPoint);
    }

    @Override
    public Interval getInterval(Vector3f axis) {
        float centerProjection = axis.x * getCenter().x + axis.y * getCenter().y + axis.z * getCenter().z;

        // Calculate the half-length of the projection
        float halfLength =
                getHalfExtents().x * Math.abs(axis.x) +
                        getHalfExtents().y * Math.abs(axis.y) +
                        getHalfExtents().z * Math.abs(axis.z);

        // Calculate the interval
        float min = centerProjection - halfLength;
        float max = centerProjection + halfLength;

        return new Interval(min, max);
    }

    @Override
    public boolean isPointInside(Vector3f point) {
        Vector3f localPoint = point.sub(center); // Transform point to local space
        for (int i = 0; i < 3; i++) {
            float distance = localPoint.dot(axis[i]);
            if (distance > halfExtents.get(i) || distance < -halfExtents.get(i)) {
                return false; // Point is outside along this axis
            }
        }
        return true; // Point is inside along all axes
    }

    @Override
    public Vector3f closestPoint(Vector3f point) {
        Vector3f result = center;
        Vector3f dir = point.sub(center);

        for (int i = 0; i < 3; ++i) {
            float distance = dir.dot(axis[i]);

            if (distance > halfExtents.get(i)) {
                distance = halfExtents.get(i);
            }
            if (distance < -halfExtents.get(i)) {
                distance = -halfExtents.get(i);
            }
            result = result.add(axis[i].mul(distance));
        }

        return result;
    }

    @Override
    public Vector3f support(Vector3f direction) {
        // Initialize variables to store the maximum dot product and the corresponding vertex
        float maxDotProduct = Float.NEGATIVE_INFINITY;
        Vector3f supportVertex = null;

        // Iterate through all the vertices of the OBB
        for (int i = 0; i < 8; i++) {
            // Calculate the vertex position using a combination of half extents and axis
            Vector3f vertex = new Vector3f(
                    center.x + halfExtents.x * ((i & 1) == 0 ? 1 : -1),
                    center.y + halfExtents.y * ((i & 2) == 0 ? 1 : -1),
                    center.z + halfExtents.z * ((i & 4) == 0 ? 1 : -1)
            );

            // Calculate the dot product of the vertex and the given direction
            float dotProduct = vertex.dot(direction);

            // Check if this vertex has a greater dot product than the current maximum
            if (dotProduct > maxDotProduct) {
                maxDotProduct = dotProduct;
                supportVertex = vertex;
            }
        }

        // Return the support vertex
        return supportVertex;
    }

    public void translate(Vector3f translation) {
        center.set(center.add(translation));
    }

    public void scale(Vector3f scaleFactor) {
        halfExtents.x *= scaleFactor.x;
        halfExtents.y *= scaleFactor.y;
        halfExtents.z *= scaleFactor.z;
    }

    public void rotate(Quaternion rotation) {
        // Rotate each axis by the given quaternion
        for (int i = 0; i < axis.length; i++) {
            axis[i] = axis[i].rotate(rotation);
        }
    }

    @Override
    public String toString() {
        return "OBB{" +
                "center=" + center +
                ", axis=" + Arrays.toString(axis) +
                ", halfExtents=" + halfExtents +
                '}';
    }

    @Override
    public int hashCode() {
        int result = Objects.hash(center, halfExtents);
        result = 31 * result + Arrays.hashCode(axis);
        return result;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        OBB obb = (OBB) o;
        return Objects.equals(center, obb.center) && Arrays.equals(axis, obb.axis) && Objects.equals(halfExtents, obb.halfExtents);
    }
}
