package collisionDetection.util.quickHull;

import com.github.quickhull3d.InternalErrorException;
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

    public static QuickHullFace create(QuickHullVertex[] vtxArray, int[] indices) {
        QuickHullFace face = new QuickHullFace();
        QuickHullHalfEdge hePrev = null;
        for (int i = 0; i < indices.length; i++) {
            QuickHullHalfEdge he = new QuickHullHalfEdge(vtxArray[indices[i]], face);
            if (hePrev != null) {
                he.setPrev(hePrev);
                hePrev.setNext(he);
            } else {
                face.he0 = he;
            }
            hePrev = he;
        }
        face.he0.setPrev(hePrev);
        assert hePrev != null;
        hePrev.setNext(face.he0);

        // compute the normal and offset
        face.computeNormalAndCentroid();
        return face;
    }

    public static QuickHullFace createTriangle(QuickHullVertex v0, QuickHullVertex v1, QuickHullVertex v2) {
        return createTriangle(v0, v1, v2, 0);
    }

    public static QuickHullFace createTriangle(QuickHullVertex v0, QuickHullVertex v1, QuickHullVertex v2, double minArea) {
        QuickHullFace face = new QuickHullFace();
        QuickHullHalfEdge he0 = new QuickHullHalfEdge(v0, face);
        QuickHullHalfEdge he1 = new QuickHullHalfEdge(v1, face);
        QuickHullHalfEdge he2 = new QuickHullHalfEdge(v2, face);

        he0.setPrev(he2);
        he0.setNext(he1);
        he1.setPrev(he0);
        he1.setNext(he2);
        he2.setPrev(he1);
        he2.setNext(he0);

        face.he0 = he0;

        // compute the normal and offset
        face.computeNormalAndCentroid(minArea);
        return face;
    }

    public void computeCentroid(Vector3f centroid) {
        centroid.setXYZ(0, 0, 0);
        QuickHullHalfEdge he = he0;
        do {
            centroid = centroid.add(he.getHead().getPnt());
            he = he.getNext();
        } while (he != he0);
        centroid.set(centroid.mul((float) 1 / numVerts));
    }

    public void computeNormal(Vector3f normal) {
        QuickHullHalfEdge he1 = he0.getNext();
        QuickHullHalfEdge he2 = he1.getNext();

        Vector3f p0 = he0.getHead().getPnt();
        Vector3f p2 = he1.getHead().getPnt();

        double d2x = p2.x - p0.x;
        double d2y = p2.y - p0.y;
        double d2z = p2.z - p0.z;

        normal.setXYZ(0, 0, 0);

        numVerts = 2;

        while (he2 != he0) {
            double d1x = d2x;
            double d1y = d2y;
            double d1z = d2z;

            p2 = he2.getHead().getPnt();
            d2x = p2.x - p0.x;
            d2y = p2.y - p0.y;
            d2z = p2.z - p0.z;

            normal.x += d1y * d2z - d1z * d2y;
            normal.y += d1z * d2x - d1x * d2z;
            normal.z += d1x * d2y - d1y * d2x;

            he2 = he2.getNext();
            numVerts++;
        }
        area = normal.length();
        normal.sub(normal.mul((float) (1 / area)));
    }

    public void computeNormal(Vector3f normal, double minArea) {
        computeNormal(normal);

        if (area < minArea) {
            // make the normal more robust by removing
            // components parallel to the longest edge

            QuickHullHalfEdge hedgeMax = null;
            double lenSqrMax = 0;
            QuickHullHalfEdge hedge = he0;
            do {
                double lenSqr = hedge.lengthSquared();
                if (lenSqr > lenSqrMax) {
                    hedgeMax = hedge;
                    lenSqrMax = lenSqr;
                }
                hedge = hedge.getNext();
            } while (hedge != he0);

            Vector3f p2 = hedgeMax.getHead().getPnt();
            Vector3f p1 = hedgeMax.getTail().getPnt();
            double lenMax = Math.sqrt(lenSqrMax);
            double ux = (p2.x - p1.x) / lenMax;
            double uy = (p2.y - p1.y) / lenMax;
            double uz = (p2.z - p1.z) / lenMax;
            double dot = normal.x * ux + normal.y * uy + normal.z * uz;
            normal.x -= dot * ux;
            normal.y -= dot * uy;
            normal.z -= dot * uz;

            normal.sub(normal.normalize());
        }
    }

    public double distanceToPlane(Vector3f p) {
        return normal.x * p.x + normal.y * p.y + normal.z * p.z - planeOffset;
    }


    public QuickHullHalfEdge findEdge(QuickHullVertex vt, QuickHullVertex vh) {
        QuickHullHalfEdge he = he0;
        do {
            if (he.getHead() == vh && he.getTail() == vt) {
                return he;
            }
            he = he.getNext();
        } while (he != he0);
        return null;
    }

    public Vector3f getCentroid() {
        return centroid;
    }


    public QuickHullHalfEdge getEdge(int i) {
        QuickHullHalfEdge he = he0;
        while (i > 0) {
            he = he.getNext();
            i--;
        }
        while (i < 0) {
            he = he.getPrev();
            i++;
        }
        return he;
    }

    public QuickHullHalfEdge getFirstEdge() {
        return he0;
    }

    public Vector3f getNormal() {
        return normal;
    }

    public void getVertexIndices(int[] idxs) {
        QuickHullHalfEdge he = he0;
        int i = 0;
        do {
            idxs[i++] = he.getHead().getIndex();
            he = he.getNext();
        } while (he != he0);
    }


    public int mergeAdjacentFace(QuickHullHalfEdge hedgeAdj, QuickHullFace[] discarded) {
        QuickHullFace oppFace = hedgeAdj.getOppositeFace();
        int numDiscarded = 0;

        discarded[numDiscarded++] = oppFace;
        oppFace.mark = DELETED;

        QuickHullHalfEdge hedgeOpp = hedgeAdj.getOpposite();

        QuickHullHalfEdge hedgeAdjPrev = hedgeAdj.getPrev();
        QuickHullHalfEdge hedgeAdjNext = hedgeAdj.getNext();
        QuickHullHalfEdge hedgeOppPrev = hedgeOpp.getPrev();
        QuickHullHalfEdge hedgeOppNext = hedgeOpp.getNext();

        while (hedgeAdjPrev.getOppositeFace() == oppFace) {
            hedgeAdjPrev = hedgeAdjPrev.getPrev();
            hedgeOppNext = hedgeOppNext.getNext();
        }

        while (hedgeAdjNext.getOppositeFace() == oppFace) {
            hedgeOppPrev = hedgeOppPrev.getPrev();
            hedgeAdjNext = hedgeAdjNext.getNext();
        }

        QuickHullHalfEdge hedge;

        for (hedge = hedgeOppNext; hedge != hedgeOppPrev.getNext(); hedge = hedge.getNext()) {
            hedge.setFace(this);
        }

        if (hedgeAdj == he0) {
            he0 = hedgeAdjNext;
        }

        // handle the half edges at the head
        QuickHullFace discardedFace;

        discardedFace = connectHalfEdges(hedgeOppPrev, hedgeAdjNext);
        if (discardedFace != null) {
            discarded[numDiscarded++] = discardedFace;
        }

        // handle the half edges at the tail
        discardedFace = connectHalfEdges(hedgeAdjPrev, hedgeOppNext);
        if (discardedFace != null) {
            discarded[numDiscarded++] = discardedFace;
        }

        computeNormalAndCentroid();
        checkConsistency();

        return numDiscarded;
    }

    public int numVertices() {
        return numVerts;
    }

    public void triangulate(QuickHullFaceList newFaces, double minArea) {
        QuickHullHalfEdge hedge;

        if (numVertices() < 4) {
            return;
        }

        QuickHullVertex v0 = he0.getHead();

        hedge = he0.getNext();
        QuickHullHalfEdge oppPrev = hedge.getOpposite();
        QuickHullFace face0 = null;

        for (hedge = hedge.getNext(); hedge != he0.getPrev(); hedge = hedge.getNext()) {
            QuickHullFace face = createTriangle(v0, hedge.getPrev().getHead(), hedge.getHead(), minArea);
            face.he0.getNext().setOpposite(oppPrev);
            face.he0.getPrev().setOpposite(hedge.getOpposite());
            oppPrev = face.he0;
            newFaces.add(face);
            if (face0 == null) {
                face0 = face;
            }
        }
        hedge = new QuickHullHalfEdge(he0.getPrev().getPrev().getHead(), this);
        hedge.setOpposite(oppPrev);

        hedge.setPrev(he0);
        hedge.getPrev().setNext(hedge);

        hedge.setNext(he0.getPrev());
        hedge.getNext().setPrev(hedge);

        computeNormalAndCentroid(minArea);
        checkConsistency();

        for (QuickHullFace face = face0; face != null; face = face.next) {
            face.checkConsistency();
        }

    }


    public double areaSquared(QuickHullHalfEdge hedge0, QuickHullHalfEdge hedge1) {
        Vector3f p0 = hedge0.getTail().getPnt();
        Vector3f p1 = hedge0.getTail().getPnt();
        Vector3f p2 = hedge1.getTail().getPnt();

        double dx1 = p1.x - p0.x;
        double dy1 = p1.y - p0.y;
        double dz1 = p1.z - p0.z;

        double dx2 = p2.x - p0.x;
        double dy2 = p2.y - p0.y;
        double dz2 = p2.z - p0.z;

        double x = dy1 * dz2 - dz1 * dy2;
        double y = dz1 * dx2 - dx1 * dz2;
        double z = dx1 * dy2 - dy1 * dx2;

        return x * x + y * y + z * z;
    }

    private void computeNormalAndCentroid() {
        computeNormal(normal);
        computeCentroid(centroid);
        planeOffset = normal.dot(centroid);
        int numv = 0;
        QuickHullHalfEdge he = he0;
        do {
            numv++;
            he = he.getNext();
        } while (he != he0);
        if (numv != numVerts) {
            throw new InternalErrorException("face numVerts=" + numVerts + " should be " + numv);
        }
    }

    private void computeNormalAndCentroid(double minArea) {
        computeNormal(normal, minArea);
        computeCentroid(centroid);
        planeOffset = normal.dot(centroid);
    }

    private QuickHullFace connectHalfEdges(QuickHullHalfEdge hedgePrev, QuickHullHalfEdge hedge) {
        QuickHullFace discardedFace = null;

        if (hedgePrev.getOppositeFace() == hedge.getOppositeFace()) { // then there is
            // a redundant
            // edge that we
            // can get rid
            // off

            QuickHullFace oppFace = hedge.getOppositeFace();
            QuickHullHalfEdge hedgeOpp;

            if (hedgePrev == he0) {
                he0 = hedge;
            }
            if (oppFace.numVertices() == 3) { // then we can get rid of the
                // opposite face altogether
                hedgeOpp = hedge.getOpposite().getPrev().getOpposite();

                oppFace.mark = DELETED;
                discardedFace = oppFace;
            } else {
                hedgeOpp = hedge.getOpposite().getNext();

                if (oppFace.he0 == hedgeOpp.getPrev()) {
                    oppFace.he0 = hedgeOpp;
                }
                hedgeOpp.setPrev(hedgeOpp.getPrev().getPrev());
                hedgeOpp.getPrev().setNext(hedgeOpp);
            }
            hedge.setPrev(hedgePrev.getPrev());
            hedge.getPrev().setNext(hedge);

            hedge.setOpposite(hedgeOpp);
            hedgeOpp.setOpposite(hedge);

            // oppFace was modified, so need to recompute
            oppFace.computeNormalAndCentroid();
        } else {
            hedgePrev.setNext(hedge);
            hedge.setPrev(hedgePrev);
        }
        return discardedFace;
    }

    void checkConsistency() {
        // do a sanity check on the face
        QuickHullHalfEdge hedge = he0;
        double maxd = 0;
        int numv = 0;

        if (numVerts < 3) {
            throw new InternalErrorException("degenerate face: ");
        }
        do {
            QuickHullHalfEdge hedgeOpp = hedge.getOpposite();
            if (hedgeOpp == null) {
                throw new InternalErrorException("face unreflected half edge ");
            } else if (hedgeOpp.getOpposite() != hedge) {
                throw new InternalErrorException("face opposite half edge has opposite ");
            }
            if (hedgeOpp.getHead() != hedge.getTail() || hedge.getHead() != hedgeOpp.getTail()) {
                throw new InternalErrorException("face half edge reflected by");
            }
            QuickHullFace oppFace = hedgeOpp.getFace();
            if (oppFace == null) {
                throw new InternalErrorException("face no face on half edge ");
            } else if (oppFace.mark == DELETED) {
                throw new InternalErrorException("face opposite face not on hull");
            }
            double d = Math.abs(distanceToPlane(hedge.getHead().getPnt()));
            if (d > maxd) {
                maxd = d;
            }
            numv++;
            hedge = hedge.getNext();
        } while (hedge != he0);

        if (numv != numVerts) {
            throw new InternalErrorException("face numVerts=" + numVerts + " should be " + numv);
        }

    }
}
