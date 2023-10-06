package collisionDetection.util.quickHull;

public class QuickHullVertexList {
    private QuickHullVertex head;

    private QuickHullVertex tail;

    /**
     * Clears this list.
     */
    public void clear() {
        head = tail = null;
    }

    /**
     * Adds a vertex to the end of this list.
     */
    public void add(QuickHullVertex vtx) {
        if (head == null) {
            head = vtx;
        } else {
            tail.setNext(vtx);
        }
        vtx.setPrev(tail);
        vtx.setNext(null);
        tail = vtx;
    }

    /**
     * Adds a chain of vertices to the end of this list.
     */
    public void addAll(QuickHullVertex vtx) {
        if (head == null) {
            head = vtx;
        } else {
            tail.setNext(vtx);
        }
        vtx.setPrev(tail);
        while (vtx.getNext() != null) {
            vtx = vtx.getNext();
        }
        tail = vtx;
    }

    /**
     * Deletes a vertex from this list.
     */
    public void delete(QuickHullVertex vtx) {
        if (vtx.getPrev() == null) {
            head = vtx.getNext();
        } else {
            vtx.getPrev().setNext(vtx.getNext());
        }
        if (vtx.getNext() == null) {
            tail = vtx.getPrev();
        } else {
            vtx.getNext().setPrev(vtx.getPrev());
        }
    }

    /**
     * Deletes a chain of vertices from this list.
     */
    public void delete(QuickHullVertex vtx1, QuickHullVertex vtx2) {
        if (vtx1.getPrev() == null) {
            head = vtx2.getNext();
        } else {
            vtx1.getPrev().setNext(vtx2.getNext());
        }
        if (vtx2.getNext() == null) {
            tail = vtx1.getPrev();
        } else {
            vtx2.getNext().setPrev(vtx1.getPrev());
        }
    }

    /**
     * Inserts a vertex into this list before another specificed vertex.
     */
    public void insertBefore(QuickHullVertex vtx, QuickHullVertex next) {
        vtx.setPrev(next.getPrev());
        if (next.getPrev() == null) {
            head = vtx;
        } else {
            next.getPrev().setNext(vtx);
        }
        vtx.setNext(next);
        next.setPrev(vtx);
    }

    /**
     * Returns the first element in this list.
     */
    public QuickHullVertex first() {
        return head;
    }

    /**
     * Returns true if this list is empty.
     */
    public boolean isEmpty() {
        return head == null;
    }
}
