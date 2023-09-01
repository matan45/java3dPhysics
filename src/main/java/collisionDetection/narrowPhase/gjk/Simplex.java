package collisionDetection.narrowPhase.gjk;

import math.Vector3f;

import java.util.ArrayList;
import java.util.List;

public class Simplex {

    private Vector3f[] points;
    private int size;
    private final List<Face> faces;

    public Simplex() {
        this.points = new Vector3f[4];
        this.size = 0;
        this.faces = new ArrayList<>();
    }

    public void addPoint(Vector3f point) {
        this.points[size++] = point;
    }

    public Vector3f getPoint(int index) {
        return this.points[index];
    }

    public int getSize() {
        return size; // Return the number of faces in the simplex
    }

    public void setPoints(Vector3f[] points) {
        this.points = points;
        size = points.length - 1;
    }

    public List<Face> getFaces() {
        return faces;
    }

    public Vector3f[] getPoints() {
        return points;
    }

    public void createFacesFromSimplex() {
        // Ensure that the face has at least three vertices to compute a normal vector.
        if (points.length < 3) {
            throw new IllegalArgumentException("A face must have at least three vertices to compute a normal vector.");
        }
        /*
        0, 1, 2,
		0, 3, 1,
		0, 2, 3,
		1, 3, 2
         */

        // Define the faces of the tetrahedron based on vertex combinations.
        // Each face is defined by three vertices.

        // Face 0: vertices 0, 1, 2
        faces.add(new Face(new Vector3f[]{
                points[0],
                points[1],
                points[2]
        }));

        // Face 1: vertices 0, 3, 1
        faces.add(new Face(new Vector3f[]{
                points[0],
                points[3],
                points[1]
        }));

        // Face 2: vertices 0, 2, 3
        faces.add(new Face(new Vector3f[]{
                points[0],
                points[2],
                points[3]
        }));

        // Face 3: vertices 1, 3, 2
        faces.add(new Face(new Vector3f[]{
                points[1],
                points[3],
                points[2]
        }));

    }
}

