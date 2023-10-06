package collisionDetection.util.quickHull;

import math.Vector3f;

public class QuickHullFace {
    protected static final int DELETED = 3;

    protected static final int NON_CONVEX = 2;

    protected static final int VISIBLE = 1;

    protected double area;

    protected QuickHullHalfEdge he0;

    protected int mark;

    protected QuickHullFace next;

    protected int numVerts;

    protected QuickHullVertex outside;

    protected double planeOffset;

    private Vector3f centroid;

    private Vector3f normal;

    public QuickHullFace() {
        normal = new Vector3f();
        centroid = new Vector3f();
        mark = VISIBLE;
    }

    public void computeCentroid(Point3d centroid) {
        centroid.setZero();
        HalfEdge he = he0;
        do {
            centroid.add(he.head().pnt);
            he = he.next;
        } while (he != he0);
        centroid.scale(1 / (double) numVerts);
    }

    private void computeNormalAndCentroid() {
        computeNormal(normal);
        computeCentroid(centroid);
        planeOffset = normal.dot(centroid);
        int numv = 0;
        HalfEdge he = he0;
        do {
            numv++;
            he = he.next;
        } while (he != he0);
        if (numv != numVerts) {
            throw new InternalErrorException("face " + getVertexString() + " numVerts=" + numVerts + " should be " + numv);
        }
    }

    private void computeNormalAndCentroid(double minArea) {
        computeNormal(normal, minArea);
        computeCentroid(centroid);
        planeOffset = normal.dot(centroid);
    }

    private Face connectHalfEdges(HalfEdge hedgePrev, HalfEdge hedge) {
        Face discardedFace = null;

        if (hedgePrev.oppositeFace() == hedge.oppositeFace()) { // then there is
            // a redundant
            // edge that we
            // can get rid
            // off

            Face oppFace = hedge.oppositeFace();
            HalfEdge hedgeOpp;

            if (hedgePrev == he0) {
                he0 = hedge;
            }
            if (oppFace.numVertices() == 3) { // then we can get rid of the
                // opposite face altogether
                hedgeOpp = hedge.getOpposite().prev.getOpposite();

                oppFace.mark = DELETED;
                discardedFace = oppFace;
            } else {
                hedgeOpp = hedge.getOpposite().next;

                if (oppFace.he0 == hedgeOpp.prev) {
                    oppFace.he0 = hedgeOpp;
                }
                hedgeOpp.prev = hedgeOpp.prev.prev;
                hedgeOpp.prev.next = hedgeOpp;
            }
            hedge.prev = hedgePrev.prev;
            hedge.prev.next = hedge;

            hedge.opposite = hedgeOpp;
            hedgeOpp.opposite = hedge;

            // oppFace was modified, so need to recompute
            oppFace.computeNormalAndCentroid();
        } else {
            hedgePrev.next = hedge;
            hedge.prev = hedgePrev;
        }
        return discardedFace;
    }

}
