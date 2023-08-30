package collisionDetection.primitive;

import collisionDetection.narrowPhase.Shape;
import collisionDetection.narrowPhase.sat.Interval;
import collisionDetection.narrowPhase.sat.SATSupport;
import math.Vector3f;

import java.util.List;

public class ConvexPolyhedron implements Shape, SATSupport {
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
            Vector3f outwardVector = new Vector3f(0, 0, 1);

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

    public static boolean isPolyhedronColliding(ConvexPolyhedron convexPolyhedron1, ConvexPolyhedron convexPolyhedron2) {
        List<Vector3f> vertices1 = convexPolyhedron1.getVertices();
        List<Vector3f> vertices2 = convexPolyhedron2.getVertices();

        // Loop through all the edges of both polyhedra
        for (int i = 0; i < vertices1.size(); i++) {
            Vector3f edgeStart = vertices1.get(i);
            Vector3f edgeEnd = vertices1.get((i + 1) % vertices1.size());

            Vector3f axis = edgeEnd.sub(edgeStart).normalize();

            if (isAxisSeparating(axis, convexPolyhedron1, convexPolyhedron2)) {
                return false;
            }
        }

        for (int i = 0; i < vertices2.size(); i++) {
            Vector3f edgeStart = vertices2.get(i);
            Vector3f edgeEnd = vertices2.get((i + 1) % vertices2.size());

            Vector3f axis = edgeEnd.sub(edgeStart).normalize();

            if (isAxisSeparating(axis, convexPolyhedron1, convexPolyhedron2)) {
                return false;
            }
        }

        return true;
    }

    private static boolean isAxisSeparating(Vector3f axis, ConvexPolyhedron convexPolyhedron1, ConvexPolyhedron convexPolyhedron2) {
        // Project the OBBs onto the axis
        Interval projection1 = convexPolyhedron1.getInterval(axis);
        Interval projection2 = convexPolyhedron2.getInterval(axis);

        // Check for separation between the intervals
        return projection1.getMax() < projection2.getMin() || projection2.getMax() < projection1.getMin();
    }


    @Override
    public String toString() {
        return "ConvexPolyhedron{" +
                "vertices=" + vertices +
                '}';
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
}
