package collisionDetection.util.quickHull;

import math.Vector3f;

public class QuickHullVertex {
    /**
     * Spatial point associated with this vertex.
     */
    private Vector3f pnt;

    /**
     * Back index into an array.
     */
    private int index;

    /**
     * List forward link.
     */
    private QuickHullVertex prev;

    /**
     * List backward link.
     */
    private QuickHullVertex next;

    /**
     * Current face that this vertex is outside of.
     */
    private QuickHullFace face;

    /**
     * Constructs a vertex and sets its coordinates to 0.
     */
    public QuickHullVertex() {
        pnt = new Vector3f();
    }

    /**
     * Constructs a vertex with the specified coordinates and index.
     */
    public QuickHullVertex(float x, float y, float z, int idx) {
        pnt = new Vector3f(x, y, z);
        index = idx;
    }

    public Vector3f getPnt() {
        return pnt;
    }

    public void setPnt(Vector3f pnt) {
        this.pnt = pnt;
    }

    public int getIndex() {
        return index;
    }

    public void setIndex(int index) {
        this.index = index;
    }

    public QuickHullVertex getPrev() {
        return prev;
    }

    public void setPrev(QuickHullVertex prev) {
        this.prev = prev;
    }

    public QuickHullVertex getNext() {
        return next;
    }

    public void setNext(QuickHullVertex next) {
        this.next = next;
    }

    public QuickHullFace getFace() {
        return face;
    }

    public void setFace(QuickHullFace face) {
        this.face = face;
    }
}
