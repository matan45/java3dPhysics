package collisionDetection.util.hull;

import math.Vector3f;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Face {
    private Vector3f[] vertices;
    private Vector3f normal;
    private List<Face> adjacentFaces;

    public Face(Vector3f[] vertices, Vector3f normal) {
        this.vertices = vertices;
        this.normal = normal;
        adjacentFaces = new ArrayList<>();
    }

    public void addAdjacentFace(Face face) {
        adjacentFaces.add(face);
    }

    public Vector3f[] getVertices() {
        return vertices;
    }

    public void setVertices(Vector3f[] vertices) {
        this.vertices = vertices;
    }

    public Vector3f getNormal() {
        return normal;
    }

    public void setNormal(Vector3f normal) {
        this.normal = normal;
    }

    public List<Face> getAdjacentFaces() {
        return adjacentFaces;
    }

    public void setAdjacentFaces(List<Face> adjacentFaces) {
        this.adjacentFaces = adjacentFaces;
    }

    @Override
    public String toString() {
        return "Face{" +
                "vertices=" + Arrays.toString(vertices) +
                ", normal=" + normal +
                ", adjacentFaces=" + adjacentFaces +
                '}';
    }
}
