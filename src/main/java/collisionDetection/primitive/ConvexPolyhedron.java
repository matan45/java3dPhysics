package collisionDetection.primitive;

import collisionDetection.narrowPhase.Shape;
import collisionDetection.narrowPhase.gjk.GJKSupport;
import collisionDetection.narrowPhase.sat.Interval;
import collisionDetection.narrowPhase.sat.SATSupport;
import collisionDetection.util.CollisionUtil;
import math.Vector3f;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

public class ConvexPolyhedron implements Shape, SATSupport, GJKSupport {
    private List<Vector3f> vertices;
    private List<Line> edges;

    public ConvexPolyhedron(List<Vector3f> vertices) {
        this.vertices = vertices;
        createEdges();
    }

    private void createEdges() {
        edges = new ArrayList<>();
        for (int i = 0; i < vertices.size(); i++) {
            Vector3f edgeStart = vertices.get(i);
            Vector3f edgeEnd = vertices.get((i + 1) % vertices.size());
            edges.add(new Line(edgeStart, edgeEnd));
        }
    }

    @Override
    public List<Vector3f> getVertices() {
        return vertices;
    }

    public void setVertices(List<Vector3f> vertices) {
        this.vertices = vertices;
        createEdges();
    }

    public List<Line> getEdges() {
        return edges;
    }

    @Override
    public boolean isPointInside(Vector3f point) {
        if (vertices.size() < 3)
            return false;
        // Initialize a ray from the point to a distant point outside the polyhedron
        Vector3f rayEndpoint = new Vector3f(point.x + 9999, point.y, point.z); // Adjust the distance as needed
        Line ray = new Line(point, rayEndpoint);

        int intersectionCount = 0;
        for (Line edge : edges) {
            if (isIntersect(ray, edge)) {
                intersectionCount++;
            }
        }
        return intersectionCount % 2 == 1;
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

    private boolean isIntersect(Line l1, Line l2) {
        // Four direction for two lines and points of other
        // line
        int dir1 = CollisionUtil.direction(l1.getStart(), l1.getEnd(), l2.getStart());
        int dir2 = CollisionUtil.direction(l1.getStart(), l1.getEnd(), l2.getEnd());
        int dir3 = CollisionUtil.direction(l2.getStart(), l2.getEnd(), l1.getStart());
        int dir4 = CollisionUtil.direction(l2.getStart(), l2.getEnd(), l1.getEnd());

        // When intersecting
        if (dir1 != dir2 && dir3 != dir4)
            return true;

        // When p2 of line2 are on the line1
        if (dir1 == 0 && l1.isPointInside(l2.getStart()))
            return true;

        // When p1 of line2 are on the line1
        if (dir2 == 0 && l1.isPointInside(l2.getEnd()))
            return true;

        // When p2 of line1 are on the line2
        if (dir3 == 0 && l2.isPointInside(l1.getStart()))
            return true;
        // When p1 of line1 are on the line2
        return dir4 == 0 && l2.isPointInside(l1.getEnd());
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
