package collisionDetection.primitive;

import collisionDetection.narrowPhase.Shape;
import collisionDetection.narrowPhase.gjk.GJKSupport;
import collisionDetection.narrowPhase.sat.Interval;
import collisionDetection.narrowPhase.sat.SATSupport;
import math.Vector3f;

import java.util.ArrayList;
import java.util.List;

public class ConvexPolyhedron implements Shape, SATSupport, GJKSupport {
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

    @Override
    public boolean isPointInside(Vector3f point) {
        for (int i = 0; i < vertices.size(); i++) {
            Vector3f v0 = vertices.get(i);
            Vector3f v1 = vertices.get((i + 1) % vertices.size());
            Vector3f edge = v1.sub(v0);
            Vector3f pointToVertex = point.sub(v0);

            // Calculate the normal of the plane defined by the edge and the point
            Vector3f normal = edge.cross(pointToVertex);

            // Use a known vector pointing outward from the polyhedron
            Vector3f outwardVector = new Vector3f(0, 1, 0);

            // Check if the point is on the "correct" side of the plane
            if (normal.dot(outwardVector) < 0) {
                return false;
            }
        }
        return true;
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
            axis.add(edgeEnd.sub(edgeStart));
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
}
