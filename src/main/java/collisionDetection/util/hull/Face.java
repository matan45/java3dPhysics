package collisionDetection.util.hull;

import math.Vector3f;

import java.util.ArrayList;
import java.util.List;

class Face {
    Vector3f[] vertices;
    Vector3f normal; // Normal vector of the face
    List<Face> adjacentFaces;

    public void addAdjacentFace(Face face) {
        if (adjacentFaces == null) {
            adjacentFaces = new ArrayList<>();
        }
        adjacentFaces.add(face);
    }

}
