package collisionDetection.util.quickHull;

public class QuickHullFaceList {
    private QuickHullFace head;

    private QuickHullFace tail;

    /**
     * Clears this list.
     */
    public void clear() {
        head = tail = null;
    }

    /**
     * Adds a vertex to the end of this list.
     */
    public void add(QuickHullFace vtx) {
        if (head == null) {
            head = vtx;
        } else {
            tail.next = vtx;
        }
        vtx.next = null;
        tail = vtx;
    }

    public QuickHullFace first() {
        return head;
    }

    /**
     * Returns true if this list is empty.
     */
    public boolean isEmpty() {
        return head == null;
    }
}
