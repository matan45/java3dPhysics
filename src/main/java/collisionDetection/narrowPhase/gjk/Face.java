package collisionDetection.narrowPhase.gjk;

import math.Vector3f;

public class Face {
    private final Vector3f[] vertices;   // Vertices of the face
    private Vector3f normal;       // Normal vector of the face
    private float distanceToOrigin; // Distance from the face to the origin

    public Face(Vector3f[] vertices) {
        this.vertices = vertices;
        computeNormalAndDistance();
    }

    private void computeNormalAndDistance() {
        // Compute the normal vector and distance from the face to the origin.
        Vector3f edge1 = vertices[1].sub(vertices[0]);
        Vector3f edge2 = vertices[2].sub(vertices[0]);
        normal = edge1.cross(edge2).normalize();
        distanceToOrigin = -vertices[0].dot(normal);
    }

    public Vector3f[] getVertices() {
        return vertices;
    }

    public Vector3f getNormal() {
        return normal;
    }

    public float getDistanceToOrigin() {
        return distanceToOrigin;
    }
}
