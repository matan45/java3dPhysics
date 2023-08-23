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

    @Override
    public String toString() {
        return "ConvexPolyhedron{" +
                "vertices=" + vertices +
                '}';
    }
}
