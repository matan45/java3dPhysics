package collisionDetection.util.quickHull;

import math.Vector3f;

public class Face {
    protected static final int DELETED = 3;

    protected static final int NON_CONVEX = 2;

    protected static final int VISIBLE = 1;

    protected double area;

    protected HalfEdge he0;

    protected int mark;

    protected Face next;

    protected int numVerts;

    protected Vertex outside;

    protected double planeOffset;

    private final Vector3f centroid;

    private final Vector3f normal;

    public Face() {
        normal = new Vector3f();
        centroid = new Vector3f();
        mark = VISIBLE;
    }


    public void computeCentroid(Vector3f centroid) {
        centroid.setXYZ(0,0,0);
        HalfEdge he = he0;
        do {
            centroid.set(centroid.add(he.head().pnt));
            he = he.next;
        } while (he != he0);
        centroid.set(centroid.mul(1 / (float) numVerts));
    }

    public void computeNormal(Vector3f normal) {
        HalfEdge he1 = he0.next;
        HalfEdge he2 = he1.next;

        Vector3f p0 = he0.head().pnt;
        Vector3f p2 = he1.head().pnt;

        double d2x = p2.x - p0.x;
        double d2y = p2.y - p0.y;
        double d2z = p2.z - p0.z;

        normal.set(normal.normalize());

        numVerts = 2;

        while (he2 != he0) {
            double d1x = d2x;
            double d1y = d2y;
            double d1z = d2z;

            p2 = he2.head().pnt;
            d2x = p2.x - p0.x;
            d2y = p2.y - p0.y;
            d2z = p2.z - p0.z;

            normal.x += (float) (d1y * d2z - d1z * d2y);
            normal.y += (float) (d1z * d2x - d1x * d2z);
            normal.z += (float) (d1x * d2y - d1y * d2x);

            he2 = he2.next;
            numVerts++;
        }
        area = normal.length();
        normal.set(normal.mul((float) (1 / area)));
    }

    public void computeNormal(Vector3f normal, double minArea) {
        computeNormal(normal);

        if (area < minArea) {
            HalfEdge hedgeMax = null;
            double lenSqrMax = 0;
            HalfEdge hedge = he0;
            do {
                double lenSqr = hedge.lengthSquared();
                if (lenSqr > lenSqrMax) {
                    hedgeMax = hedge;
                    lenSqrMax = lenSqr;
                }
                hedge = hedge.next;
            } while (hedge != he0);

            assert hedgeMax != null;
            Vector3f p2 = hedgeMax.head().pnt;
            Vector3f p1 = hedgeMax.tail().pnt;
            double lenMax = Math.sqrt(lenSqrMax);
            double ux = (p2.x - p1.x) / lenMax;
            double uy = (p2.y - p1.y) / lenMax;
            double uz = (p2.z - p1.z) / lenMax;
            double dot = normal.x * ux + normal.y * uy + normal.z * uz;
            normal.x -= (float) (dot * ux);
            normal.y -= (float) (dot * uy);
            normal.z -= (float) (dot * uz);

            normal.set(normal.normalize());
        }
    }

    public double distanceToPlane(Vector3f p) {
        return normal.x * p.x + normal.y * p.y + normal.z * p.z - planeOffset;
    }

    public Vector3f getCentroid() {
        return centroid;
    }

    public HalfEdge getEdge(int i) {
        HalfEdge he = he0;
        while (i > 0) {
            he = he.next;
            i--;
        }
        while (i < 0) {
            he = he.prev;
            i++;
        }
        return he;
    }

    public HalfEdge getFirstEdge() {
        return he0;
    }


    public Vector3f getNormal() {
        return normal;
    }

    public String getVertexString() {
        String s = null;
        HalfEdge he = he0;
        do {
            if (s == null) {
                s = "" + he.head().index;
            } else {
                s += " " + he.head().index;
            }
            he = he.next;
        } while (he != he0);
        return s;
    }

    public int mergeAdjacentFace(HalfEdge hedgeAdj, Face[] discarded) {
        Face oppFace = hedgeAdj.oppositeFace();
        int numDiscarded = 0;

        discarded[numDiscarded++] = oppFace;
        oppFace.mark = DELETED;

        HalfEdge hedgeOpp = hedgeAdj.getOpposite();

        HalfEdge hedgeAdjPrev = hedgeAdj.prev;
        HalfEdge hedgeAdjNext = hedgeAdj.next;
        HalfEdge hedgeOppPrev = hedgeOpp.prev;
        HalfEdge hedgeOppNext = hedgeOpp.next;

        while (hedgeAdjPrev.oppositeFace() == oppFace) {
            hedgeAdjPrev = hedgeAdjPrev.prev;
            hedgeOppNext = hedgeOppNext.next;
        }

        while (hedgeAdjNext.oppositeFace() == oppFace) {
            hedgeOppPrev = hedgeOppPrev.prev;
            hedgeAdjNext = hedgeAdjNext.next;
        }

        HalfEdge hedge;

        for (hedge = hedgeOppNext; hedge != hedgeOppPrev.next; hedge = hedge.next) {
            hedge.face = this;
        }

        if (hedgeAdj == he0) {
            he0 = hedgeAdjNext;
        }

        // handle the half edges at the head
        Face discardedFace;

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

    public void computeNormalAndCentroid() {
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

    public void computeNormalAndCentroid(double minArea) {
        computeNormal(normal, minArea);
        computeCentroid(centroid);
        planeOffset = normal.dot(centroid);
    }

    private Face connectHalfEdges(HalfEdge hedgePrev, HalfEdge hedge) {
        Face discardedFace = null;

        if (hedgePrev.oppositeFace() == hedge.oppositeFace()) {

            Face oppFace = hedge.oppositeFace();
            HalfEdge hedgeOpp;

            if (hedgePrev == he0) {
                he0 = hedge;
            }
            if (oppFace.numVertices() == 3) {
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

    void checkConsistency() {
        // do a sanity check on the face
        HalfEdge hedge = he0;
        double maxd = 0;
        int numv = 0;

        if (numVerts < 3) {
            throw new InternalErrorException("degenerate face: " + getVertexString());
        }
        do {
            HalfEdge hedgeOpp = hedge.getOpposite();
            if (hedgeOpp == null) {
                throw new InternalErrorException("face " + getVertexString() + ": " + "unreflected half edge " + hedge.getVertexString());
            } else if (hedgeOpp.getOpposite() != hedge) {
                throw new InternalErrorException("face " + getVertexString() + ": " + "opposite half edge " + hedgeOpp.getVertexString() + " has opposite "
                        + hedgeOpp.getOpposite().getVertexString());
            }
            if (hedgeOpp.head() != hedge.tail() || hedge.head() != hedgeOpp.tail()) {
                throw new InternalErrorException("face " + getVertexString() + ": " + "half edge " + hedge.getVertexString() + " reflected by " + hedgeOpp.getVertexString());
            }
            Face oppFace = hedgeOpp.face;
            if (oppFace == null) {
                throw new InternalErrorException("face " + getVertexString() + ": " + "no face on half edge " + hedgeOpp.getVertexString());
            } else if (oppFace.mark == DELETED) {
                throw new InternalErrorException("face " + getVertexString() + ": " + "opposite face " + oppFace.getVertexString() + " not on hull");
            }
            double d = Math.abs(distanceToPlane(hedge.head().pnt));
            if (d > maxd) {
                maxd = d;
            }
            numv++;
            hedge = hedge.next;
        } while (hedge != he0);

        if (numv != numVerts) {
            throw new InternalErrorException("face " + getVertexString() + " numVerts=" + numVerts + " should be " + numv);
        }

    }
}
