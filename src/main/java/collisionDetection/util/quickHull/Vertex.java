package collisionDetection.util.quickHull;

import math.Vector3f;

public class Vertex {
    Vector3f pnt;

    int index;

    Vertex prev;

    Vertex next;

    Face face;

    public Vertex() {
        pnt = new Vector3f();
    }

    public Vertex(float x, float y, float z, int idx) {
        pnt = new Vector3f(x, y, z);
        index = idx;
    }

}
