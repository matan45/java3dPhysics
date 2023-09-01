package collisionDetection.narrowPhase.gjk;

import math.Vector3f;

import java.util.ArrayList;
import java.util.List;

public class Simplex {

    private final Vector3f[] points;
    private final List<Face> faces;// List of faces in the simplex
    private int rank;
    private int size;

    public Simplex() {
        this.points = new Vector3f[4];
        this.rank = 1;
        this.size = 0;
        this.faces = new ArrayList<>();
    }

    public void addPoint(Vector3f point) {
        this.points[size++] = point;
        rank++;
        // Check if you have enough points to form a face (e.g., an edge or a triangle)
        if (rank == 2) {
            // Create an edge (a 2D face) using the first two points
            Vector3f point1 = points[0];
            Vector3f point2 = points[1];
            Vector3f[] edgeVertices = {point1, point2};
            Face edgeFace = new Face(edgeVertices);
            faces.add(edgeFace);
        } else if (rank == 3) {
            // Create a triangle (a 3D face) using the first three points
            Vector3f point1 = points[0];
            Vector3f point2 = points[1];
            Vector3f point3 = points[2];
            Vector3f[] triangleVertices = {point1, point2, point3};
            Face triangleFace = new Face(triangleVertices);
            faces.add(triangleFace);
        }
    }

    public Vector3f getPoint(int index) {
        return this.points[index];
    }

    public int getRank() {
        return this.rank;
    }

    public boolean containsOrigin() {
        // Check if the origin is inside the simplex using the vertices' orientations.
        if (rank == 2) {
            // For line segment, check if the origin is on the line.
            Vector3f a = points[0];
            Vector3f b = points[1];
            Vector3f ao = new Vector3f(0, 0, 0).sub(a);
            Vector3f ab = b.sub(a);

            // Check if the origin is on the line segment.
            return ab.dot(ao) <= 0;
        } else if (rank == 3) {
            // For triangle, check if the origin is inside the triangle.
            Vector3f a = points[0];
            Vector3f b = points[1];
            Vector3f c = points[2];
            Vector3f ao = new Vector3f(0, 0, 0).sub(a);
            Vector3f ab = b.sub(a);
            Vector3f ac = c.sub(a);

            // Compute normal of the triangle.
            Vector3f normal = ab.cross(ac);

            // Check the orientation of the origin with respect to the triangle's normal.
            return normal.dot(ao) >= 0;
        } else if (rank == 4) {
            // For tetrahedron, check if the origin is inside the tetrahedron.
            Vector3f a = points[0];
            Vector3f b = points[1];
            Vector3f c = points[2];
            Vector3f d = points[3];

            // Check the orientation of the origin with respect to the tetrahedron's normals.
            Vector3f abcNormal = b.sub(a).cross(c.sub(a));
            Vector3f abdNormal = b.sub(a).cross(d.sub(a));
            Vector3f acdNormal = c.sub(a).cross(d.sub(a));
            Vector3f bcdNormal = c.sub(b).cross(d.sub(b));

            Vector3f ao = new Vector3f(0, 0, 0).sub(a);

            return abcNormal.dot(ao) >= 0 && abdNormal.dot(ao) >= 0 && acdNormal.dot(ao) >= 0 && bcdNormal.dot(ao) >= 0;
        }

        return false;
    }

    public Face findClosestFaceToOrigin() {
        // Ensure the simplex has at least 3 points (a triangle) to define faces.
        if (rank < 3) {
            throw new IllegalStateException("Simplex must have at least 3 points to find faces.");
        }

        // Iterate over all faces formed by the simplex's vertices and find the closest face.
        float minDistance = Float.MAX_VALUE;
        Face closestFace = null;

        for (int i = 0; i < rank; i++) {
            for (int j = i + 1; j < rank; j++) {
                for (int k = j + 1; k < rank; k++) {
                    Vector3f[] triangleVertices = {points[i], points[j], points[k]};
                    Face currentFace = new Face(triangleVertices);

                    // Check if the origin is in front of the face (i.e., on the side opposite to the normal).
                    if (currentFace.getNormal().dot(triangleVertices[0]) <= 0) {
                        float distanceToOrigin = currentFace.getDistanceToOrigin();
                        if (distanceToOrigin < minDistance) {
                            minDistance = distanceToOrigin;
                            closestFace = currentFace;
                        }
                    }
                }
            }
        }

        if (closestFace == null) {
            throw new IllegalStateException("No closest face found.");
        }

        return closestFace;
    }

    public int getSize() {
        return faces.size(); // Return the number of faces in the simplex
    }

    public Face getFace(int index) {
        return faces.get(index); // Return the face at the specified index
    }

}
