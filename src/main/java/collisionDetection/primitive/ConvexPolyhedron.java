package collisionDetection.primitive;

import collisionDetection.narrowPhase.Shape;
import collisionDetection.narrowPhase.gjk.GJKSupport;
import collisionDetection.narrowPhase.sat.Interval;
import collisionDetection.narrowPhase.sat.SATSupport;
import math.Vector3f;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

public class ConvexPolyhedron implements Shape, SATSupport, GJKSupport {
    private List<Vector3f> vertices;

    public ConvexPolyhedron(List<Vector3f> vertices) {
        this.vertices = vertices;
    }

    @Override
    public List<Vector3f> getVertices() {
        return vertices;
    }

    public void setVertices(List<Vector3f> vertices) {
        this.vertices = vertices;
    }

    @Override
    public boolean isPointInside(Vector3f point) {
        boolean isInside = true;
        // Iterate through the vertices of the polyhedron.
        for (Vector3f vertex : vertices) {
            // Calculate the vector from the current vertex to the test point.
            Vector3f vectorToVertex = vertex.sub(point);

            // Check if the dot product between the vector and the vertex normal
            // is positive for all vertices. If it is, the point is inside.
            if (vectorToVertex.dot(vertex) < 0) {
                isInside = false;
            }
        }
        return isInside;
    }


    @Override
    public Vector3f closestPoint(Vector3f point) {
        Vector3f closestPoint = null;
        float minDistance = Float.POSITIVE_INFINITY;

        for (Vector3f vertex : vertices) {
            float distance = vertex.distance(point);
            if (distance < minDistance) {
                minDistance = distance;
                closestPoint = vertex;
            }
        }

        return closestPoint;
    }

    @Override
    public Interval getInterval(Vector3f axis) {
        float min = Float.MAX_VALUE;
        float max = Float.MIN_VALUE;

        for (Vector3f vertex : vertices) {
            float projection = vertex.dot(axis);
            min = Math.min(min, projection);
            max = Math.max(max, projection);
        }
        return new Interval(min, max);
    }

    @Override
    public List<Vector3f> getAxis() {
        List<Vector3f> axis = new ArrayList<>();

        for (int i = 0; i < vertices.size(); i++) {
            Vector3f edgeStart = vertices.get(i);
            Vector3f edgeEnd = vertices.get((i + 1) % vertices.size());
            axis.add(edgeEnd.sub(edgeStart).normalize());
        }
        return axis;
    }

    @Override
    public Vector3f support(Vector3f direction) {
        Vector3f supportPoint = vertices.get(0);
        float maxProjection = vertices.get(0).dot(direction);

        for (Vector3f vertex : vertices) {
            float projection = vertex.dot(direction);
            if (projection > maxProjection) {
                maxProjection = projection;
                supportPoint.set(vertex);
            }
        }

        return supportPoint;
    }

    @Override
    public String toString() {
        return "ConvexPolyhedron{" +
                "vertices=" + vertices +
                '}';
    }

    @Override
    public int hashCode() {
        return Objects.hash(vertices);
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        ConvexPolyhedron that = (ConvexPolyhedron) o;
        return Objects.equals(vertices, that.vertices);
    }
}
