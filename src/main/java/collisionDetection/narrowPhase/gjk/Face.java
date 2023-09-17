package collisionDetection.narrowPhase.gjk;

import collisionDetection.narrowPhase.collisionResult.CollisionResult;
import math.Vector3f;

import java.util.*;

public class Face {
    private final Vector3f[] vertices;   // Vertices of the face
    private Vector3f normal;       // Normal vector of the face
    private float distanceToOrigin; // Distance from the face to the origin

    public Face(Vector3f[] vertices) {
        this.vertices = vertices;
        computeNormalAndDistance();
    }

    public Face() {
        this.vertices = new Vector3f[3];
    }

    private void computeNormalAndDistance() {
        // Ensure that the face has at least three vertices to compute a normal vector.
        if (vertices.length < 3) {
            throw new IllegalArgumentException("A face must have at least three vertices to compute a normal vector.");
        }
        // Compute the normal vector and distance from the face to the origin.
        Vector3f edge1 = vertices[1].sub(vertices[0]);
        Vector3f edge2 = vertices[2].sub(vertices[0]);
        normal = edge1.cross(edge2).normalize();
        distanceToOrigin = vertices[0].dot(normal);
        if (distanceToOrigin < 0) {
            distanceToOrigin *= -1;
            normal.set(normal.negate());
        }
    }

    public CollisionResult extractCollisionResult() {
        // Calculate the penetration depth as the signed distance from the origin (0,0,0)
        // to the plane defined by the face (using the face's normal vector)
        float penetrationDepth = calculatePenetrationDepth();

        // The collision normal is simply the normalized normal vector of the face
        Vector3f collisionNormal = normal;

        // Calculate contact points by finding the vertices of the face
        // These vertices represent the contact points
        List<Vector3f> contactPoints = new ArrayList<>();
        Collections.addAll(contactPoints, vertices);

        // Create a new CollisionResult object with the extracted collision information
        return new CollisionResult(true, collisionNormal, penetrationDepth, contactPoints);
    }

    private float calculatePenetrationDepth() {
        // You can calculate the penetration depth by using the dot product of
        // the origin (0,0,0) to any point on the plane (e.g., a vertex)
        // and the face's normal vector, then taking the absolute value.
        // The absolute value ensures that the penetration depth is always positive.
        Vector3f pointOnPlane = vertices[0]; // Choose one of the vertices as a reference
        Vector3f originToPlane = pointOnPlane.negate(); // Assuming origin is at (0,0,0)
        return Math.abs(originToPlane.dot(normal));
    }

    public float distanceToPoint(Vector3f point) {
        // Calculate the vector from the point to any point on the plane (e.g., a vertex)
        Vector3f pointOnPlane = vertices[0]; // Choose one of the vertices as a reference
        Vector3f pointToPlane = pointOnPlane.sub(point);

        // Calculate the dot product of the vector to the plane and the plane's normal
        float distance = pointToPlane.dot(normal);

        // The absolute value of the distance represents the distance from the point to the plane
        return Math.abs(distance);
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

    @Override
    public String toString() {
        return "Face{" +
                "vertices=" + Arrays.toString(vertices) +
                ", normal=" + normal +
                ", distanceToOrigin=" + distanceToOrigin +
                '}';
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Face face = (Face) o;
        return Float.compare(distanceToOrigin, face.distanceToOrigin) == 0 && Arrays.equals(vertices, face.vertices) && Objects.equals(normal, face.normal);
    }

    @Override
    public int hashCode() {
        int result = Objects.hash(normal, distanceToOrigin);
        result = 31 * result + Arrays.hashCode(vertices);
        return result;
    }
}
