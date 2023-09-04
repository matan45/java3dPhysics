package collisionDetection.primitive;

import collisionDetection.narrowPhase.Shape;
import collisionDetection.narrowPhase.gjk.GJKSupport;
import collisionDetection.narrowPhase.sat.Interval;
import collisionDetection.narrowPhase.sat.SATSupport;
import math.Vector3f;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

public class Triangle implements Shape, SATSupport, GJKSupport {
    private Vector3f vertex1;
    private Vector3f vertex2;
    private Vector3f vertex3;

    public Triangle(Vector3f vertex1, Vector3f vertex2, Vector3f vertex3) {
        this.vertex1 = vertex1;
        this.vertex2 = vertex2;
        this.vertex3 = vertex3;
    }

    public List<Vector3f> getVertices() {
        List<Vector3f> vertices = new ArrayList<>();
        vertices.add(vertex1);
        vertices.add(vertex2);
        vertices.add(vertex3);
        return vertices;
    }

    public Vector3f getVertex1() {
        return vertex1;
    }

    public void setVertex1(Vector3f vertex1) {
        this.vertex1 = vertex1;
    }

    public Vector3f getVertex2() {
        return vertex2;
    }

    public void setVertex2(Vector3f vertex2) {
        this.vertex2 = vertex2;
    }

    public Vector3f getVertex3() {
        return vertex3;
    }

    public void setVertex3(Vector3f vertex3) {
        this.vertex3 = vertex3;
    }

    public Vector3f getEdge1() {
        return vertex2.sub(vertex1);
    }

    public Vector3f getEdge2() {
        return vertex3.sub(vertex2);
    }

    public Vector3f getEdge3() {
        return vertex1.sub(vertex3);
    }

    private Plane fromTriangle() {
        Plane result = new Plane(new Vector3f(), 0);
        result.setNormal(vertex2.sub(vertex1).cross(vertex3.sub(vertex1)));
        result.setDistance(result.getNormal().dot(vertex1));
        return result;
    }

    @Override
    public boolean isPointInside(Vector3f point) {
        Vector3f a = vertex1.sub(point);
        Vector3f b = vertex2.sub(point);
        Vector3f c = vertex3.sub(point);

        Vector3f normPBC = b.cross(c); // Normal of PBC (u)
        Vector3f normPCA = c.cross(a); // Normal of PCA (v)
        Vector3f normPAB = a.cross(b); // Normal of PAB (w)

        if (normPBC.dot(normPCA) < 0.0f) {
            return false;
        }
        return !(normPBC.dot(normPAB) < 0.0f);
    }


    @Override
    public Vector3f closestPoint(Vector3f point) {
        Plane plane = fromTriangle();
        Vector3f closest = plane.closestPoint(point);

        if (isPointInside(closest)) {
            return closest;
        }

        Vector3f c1 = ClosestPoint(vertex1, vertex2, point); // Line AB
        Vector3f c2 = ClosestPoint(vertex2, vertex3, point); // Line AB
        Vector3f c3 = ClosestPoint(vertex3, vertex1, point); // Line AB

        float magSq1 = point.sub(c1).lengthSquared();
        float magSq2 = point.sub(c2).lengthSquared();
        float magSq3 = point.sub(c3).lengthSquared();

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


    @Override
    public Interval getInterval(Vector3f axis) {
        // Project the triangle vertices onto the axis
        float projection1 = axis.dot(getVertex1());
        float projection2 = axis.dot(getVertex2());
        float projection3 = axis.dot(getVertex3());

        // Calculate the minimum and maximum values of the projection
        float min = Math.min(Math.min(projection1, projection2), projection3);
        float max = Math.max(Math.max(projection1, projection2), projection3);

        return new Interval(min, max);
    }

    @Override
    public List<Vector3f> getAxis() {
        return List.of(getEdge1(),
                getEdge2(),
                getEdge3());
    }

    public Vector3f calculateTriangleNormal() {
        // Calculate triangle normal using cross product
        Vector3f edge1 = getEdge1();
        Vector3f edge2 = getEdge2();
        return edge1.cross(edge2).normalize();
    }

    @Override
    public Vector3f support(Vector3f direction) {
        float dot1 = vertex1.dot(direction);
        float dot2 = vertex2.dot(direction);
        float dot3 = vertex3.dot(direction);

        Vector3f supportPoint;

        if (dot1 >= dot2 && dot1 >= dot3) {
            supportPoint = new Vector3f(vertex1);
        } else if (dot2 >= dot1 && dot2 >= dot3) {
            supportPoint = new Vector3f(vertex2);
        } else {
            supportPoint = new Vector3f(vertex3);
        }

        return supportPoint;
    }

    @Override
    public String toString() {
        return "Triangle{" +
                "vertex1=" + vertex1 +
                ", vertex2=" + vertex2 +
                ", vertex3=" + vertex3 +
                '}';
    }

    @Override
    public int hashCode() {
        return Objects.hash(vertex1, vertex2, vertex3);
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Triangle triangle = (Triangle) o;
        return Objects.equals(vertex1, triangle.vertex1) && Objects.equals(vertex2, triangle.vertex2) && Objects.equals(vertex3, triangle.vertex3);
    }
}
