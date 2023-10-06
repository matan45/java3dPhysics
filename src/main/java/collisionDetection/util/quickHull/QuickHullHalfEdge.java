package collisionDetection.util.quickHull;

public class QuickHullHalfEdge {

    private QuickHullVertex vertex;
    private QuickHullFace face;
    private QuickHullHalfEdge next;
    private QuickHullHalfEdge prev;
    private QuickHullHalfEdge opposite;

    public QuickHullHalfEdge(QuickHullVertex vertex, QuickHullFace face) {
        this.vertex = vertex;
        this.face = face;
    }

    public QuickHullHalfEdge() {
    }

    public QuickHullHalfEdge getNext() {
        return next;
    }

    public void setNext(QuickHullHalfEdge next) {
        this.next = next;
    }

    public QuickHullHalfEdge getPrev() {
        return prev;
    }

    public void setPrev(QuickHullHalfEdge prev) {
        this.prev = prev;
    }

    public QuickHullFace getFace() {
        return face;
    }

    public void setFace(QuickHullFace face) {
        this.face = face;
    }

    public QuickHullHalfEdge getOpposite() {
        return opposite;
    }

    public void setOpposite(QuickHullHalfEdge opposite) {
        this.opposite = opposite;
        opposite.opposite = this;
    }


    public QuickHullVertex getHead() {
        return vertex;
    }


    public QuickHullVertex getTail() {
        return prev != null ? prev.vertex : null;
    }

    public QuickHullFace getOppositeFace() {
        return opposite != null ? opposite.face : null;
    }


    public float length() {
        if (getTail() != null) {
            return getHead().getPnt().distance(getTail().getPnt());
        } else {
            return -1;
        }
    }


    public double lengthSquared() {
        if (getTail() != null) {
            return getHead().getPnt().distanceSquared(getTail().getPnt());
        } else {
            return -1;
        }
    }
}
