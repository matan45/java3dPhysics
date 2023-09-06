package collisionDetection.primitive.terrain;

import collisionDetection.primitive.Triangle;
import math.Vector2f;
import math.Vector3f;

import java.util.Objects;

public class TerrainTriangle {
    private Vector3f vertex1;
    private Vector3f vertex2;
    private Vector3f vertex3;
    private Vector3f normal;
    private Vector2f gridPosition;

    public TerrainTriangle(Vector3f vertex1, Vector3f vertex2, Vector3f vertex3, Vector2f gridPosition) {
        this.vertex1 = vertex1;
        this.vertex2 = vertex2;
        this.vertex3 = vertex3;
        this.gridPosition = gridPosition;
        this.normal = calculateFaceNormal(vertex1, vertex2, vertex3);
    }

    public Triangle toTriangle() {
        return new Triangle(vertex1, vertex2, vertex3);
    }

    private Vector3f calculateFaceNormal(Vector3f vertex1, Vector3f vertex2, Vector3f vertex3) {
        // Calculate triangle normal using cross product
        Vector3f edge1 = vertex2.sub(vertex1);
        Vector3f edge2 = vertex3.sub(vertex2);
        return edge1.cross(edge2).normalize();
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

    public Vector3f getNormal() {
        return normal;
    }

    public void setNormal(Vector3f normal) {
        this.normal = normal;
    }

    public Vector2f getGridPosition() {
        return gridPosition;
    }

    public void setGridPosition(Vector2f gridPosition) {
        this.gridPosition = gridPosition;
    }

    @Override
    public String toString() {
        return "TerrainTriangle{" +
                "vertex1=" + vertex1 +
                ", vertex2=" + vertex2 +
                ", vertex3=" + vertex3 +
                ", normal=" + normal +
                ", gridPosition=" + gridPosition +
                '}';
    }

    @Override
    public int hashCode() {
        return Objects.hash(vertex1, vertex2, vertex3, normal, gridPosition);
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        TerrainTriangle that = (TerrainTriangle) o;
        return Objects.equals(vertex1, that.vertex1) &&
                Objects.equals(vertex2, that.vertex2) &&
                Objects.equals(vertex3, that.vertex3) &&
                Objects.equals(normal, that.normal) &&
                Objects.equals(gridPosition, that.gridPosition);
    }
}
