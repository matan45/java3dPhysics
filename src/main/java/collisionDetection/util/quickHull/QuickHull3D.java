package collisionDetection.util.quickHull;

import math.Vector3f;

import java.util.ArrayList;
import java.util.List;

public class QuickHull3D {

    private static final double DOUBLE_PREC = 2.2204460492503131e-16;
    private static final int NONCONVEX_WRT_LARGER_FACE = 1;
    private static final int NONCONVEX = 2;

    private List<QuickHullVertex> hullVertices;
    private QuickHullVertex[] maxVtxs = new QuickHullVertex[3];
    private QuickHullVertex[] minVtxs = new QuickHullVertex[3];
    private QuickHullVertexList unclaimed = new QuickHullVertexList();
    private QuickHullVertexList claimed = new QuickHullVertexList();

    private  List<QuickHullHalfEdge> horizon = new ArrayList<>(16);
    private QuickHullFaceList newFaces = new QuickHullFaceList();
    // estimated size of the point set
    private double maxAxisLength;
    private double tolerance;

    public void build(List<Vector3f> points) {
        if (points.size() < 4) {
            throw new IllegalArgumentException("Less than four input points specified");
        }
        setPoints(points);
        buildHull();
    }

    private void setPoints(List<Vector3f> points) {
        hullVertices = new ArrayList<>(points.size());
        for (Vector3f point : points) {
            QuickHullVertex quickHullVertex = new QuickHullVertex();
            quickHullVertex.setPnt(point);
            hullVertices.add(quickHullVertex);
        }
    }

    private void buildHull() {
        QuickHullVertex eyeVtx;

        computeMaxAndMin();
        createInitialSimplex();
        while ((eyeVtx = nextPointToAdd()) != null) {
            addPointToHull(eyeVtx);
        }
        reindexFacesAndVertices();
    }

    private QuickHullVertex nextPointToAdd() {
        if (!claimed.isEmpty()) {
            QuickHullFace eyeFace = claimed.first().getFace();
            QuickHullVertex eyeVtx = null;
            double maxDist = 0;
            for (QuickHullVertex vtx = eyeFace.outside; vtx != null && vtx.getFace() == eyeFace; vtx = vtx.getNext()) {
                double dist = eyeFace.distanceToPlane(vtx.getPnt());
                if (dist > maxDist) {
                    maxDist = dist;
                    eyeVtx = vtx;
                }
            }
            return eyeVtx;
        } else {
            return null;
        }
    }

    protected void reindexFacesAndVertices() {
        for (int i = 0; i < numPoints; i++) {
            pointBuffer[i].index = -1;
        }
        // remove inactive faces and mark active vertices
        numFaces = 0;
        for (Iterator it = faces.iterator(); it.hasNext();) {
            Face face = (Face) it.next();
            if (face.mark != Face.VISIBLE) {
                it.remove();
            } else {
                markFaceVertices(face, 0);
                numFaces++;
            }
        }
        // reindex vertices
        numVertices = 0;
        for (int i = 0; i < numPoints; i++) {
            Vertex vtx = pointBuffer[i];
            if (vtx.index == 0) {
                vertexPointIndices[numVertices] = i;
                vtx.index = numVertices++;
            }
        }
    }

    private void markFaceVertices(Face face, int mark) {
        HalfEdge he0 = face.getFirstEdge();
        HalfEdge he = he0;
        do {
            he.head().index = mark;
            he = he.next;
        } while (he != he0);
    }

    private void addPointToHull(QuickHullVertex eyeVtx) {
        horizon.clear();
        unclaimed.clear();

        removePointFromFace(eyeVtx, eyeVtx.getFace());
        calculateHorizon(eyeVtx.getPnt(), null, eyeVtx.getFace(), horizon);
        newFaces.clear();
        addNewFaces(newFaces, eyeVtx, horizon);

        // first merge pass ... merge faces which are non-convex
        // as determined by the larger face

        for (Face face = newFaces.first(); face != null; face = face.next) {
            if (face.mark == Face.VISIBLE) {
                while (doAdjacentMerge(face, NONCONVEX_WRT_LARGER_FACE)) ;
            }
        }
        // second merge pass ... merge faces which are non-convex
        // wrt either face
        for (Face face = newFaces.first(); face != null; face = face.next) {
            if (face.mark == Face.NON_CONVEX) {
                face.mark = Face.VISIBLE;
                while (doAdjacentMerge(face, NONCONVEX)) ;
            }
        }
        resolveUnclaimedPoints(newFaces);
    }

    protected void resolveUnclaimedPoints(FaceList newFaces) {
        Vertex vtxNext = unclaimed.first();
        for (Vertex vtx = vtxNext; vtx != null; vtx = vtxNext) {
            vtxNext = vtx.next;

            double maxDist = tolerance;
            Face maxFace = null;
            for (Face newFace = newFaces.first(); newFace != null; newFace = newFace.next) {
                if (newFace.mark == Face.VISIBLE) {
                    double dist = newFace.distanceToPlane(vtx.pnt);
                    if (dist > maxDist) {
                        maxDist = dist;
                        maxFace = newFace;
                    }
                    if (maxDist > 1000 * tolerance) {
                        break;
                    }
                }
            }
            if (maxFace != null) {
                addPointToFace(vtx, maxFace);
                if (LOG.isDebugEnabled() && vtx.index == findIndex) {
                    LOG.debug(findIndex + " CLAIMED BY " + maxFace.getVertexString());
                }
            } else {
                if (LOG.isDebugEnabled() && vtx.index == findIndex) {
                    LOG.debug(findIndex + " DISCARDED");
                }
            }
        }
    }

    private boolean doAdjacentMerge(Face face, int mergeType) {
        HalfEdge hedge = face.he0;

        boolean convex = true;
        do {
            Face oppFace = hedge.oppositeFace();
            boolean merge = false;

            if (mergeType == NONCONVEX) { // then merge faces if they are
                // definitively non-convex
                if (oppFaceDistance(hedge) > -tolerance || oppFaceDistance(hedge.opposite) > -tolerance) {
                    merge = true;
                }
            } else {
                // mergeType == NONCONVEX_WRT_LARGER_FACE
                // merge faces if they are parallel or non-convex
                // wrt to the larger face; otherwise, just mark
                // the face non-convex for the second pass.
                if (face.area > oppFace.area) {
                    if (oppFaceDistance(hedge) > -tolerance) {
                        merge = true;
                    } else if (oppFaceDistance(hedge.opposite) > -tolerance) {
                        convex = false;
                    }
                } else {
                    if (oppFaceDistance(hedge.opposite) > -tolerance) {
                        merge = true;
                    } else if (oppFaceDistance(hedge) > -tolerance) {
                        convex = false;
                    }
                }
            }

            if (merge) {

                int numd = face.mergeAdjacentFace(hedge, discardedFaces);
                for (int i = 0; i < numd; i++) {
                    deleteFacePoints(discardedFaces[i], face);
                }

                return true;
            }
            hedge = hedge.next;
        } while (hedge != face.he0);
        if (!convex) {
            face.mark = Face.NON_CONVEX;
        }
        return false;
    }

    protected double oppFaceDistance(HalfEdge he) {
        return he.face.distanceToPlane(he.opposite.face.getCentroid());
    }

    protected void addNewFaces(QuickHullFaceList newFaces, QuickHullVertex eyeVtx, Vector horizon) {
        newFaces.clear();

        HalfEdge hedgeSidePrev = null;
        HalfEdge hedgeSideBegin = null;

        for (Iterator it = horizon.iterator(); it.hasNext();) {
            HalfEdge horizonHe = (HalfEdge) it.next();
            HalfEdge hedgeSide = addAdjoiningFace(eyeVtx, horizonHe);
            if (LOG.isDebugEnabled()) {
                LOG.debug("new face: " + hedgeSide.face.getVertexString());
            }
            if (hedgeSidePrev != null) {
                hedgeSide.next.setOpposite(hedgeSidePrev);
            } else {
                hedgeSideBegin = hedgeSide;
            }
            newFaces.add(hedgeSide.getFace());
            hedgeSidePrev = hedgeSide;
        }
        hedgeSideBegin.next.setOpposite(hedgeSidePrev);
    }

    protected void calculateHorizon(Point3d eyePnt, HalfEdge edge0, Face face, Vector horizon) {
        // oldFaces.add (face);
        deleteFacePoints(face, null);
        face.mark = Face.DELETED;

        HalfEdge edge;
        if (edge0 == null) {
            edge0 = face.getEdge(0);
            edge = edge0;
        } else {
            edge = edge0.getNext();
        }
        do {
            Face oppFace = edge.oppositeFace();
            if (oppFace.mark == Face.VISIBLE) {
                if (oppFace.distanceToPlane(eyePnt) > tolerance) {
                    calculateHorizon(eyePnt, edge.getOpposite(), oppFace, horizon);
                } else {
                    horizon.add(edge);
                }
            }
            edge = edge.getNext();
        } while (edge != edge0);
    }

    private void removePointFromFace(QuickHullVertex vtx, QuickHullFace face) {
        if (vtx == face.outside) {
            if (vtx.next != null && vtx.next.face == face) {
                face.outside = vtx.next;
            } else {
                face.outside = null;
            }
        }
        claimed.delete(vtx);
    }

    protected void deleteFacePoints(Face face, Face absorbingFace) {
        Vertex faceVtxs = removeAllPointsFromFace(face);
        if (faceVtxs != null) {
            if (absorbingFace == null) {
                unclaimed.addAll(faceVtxs);
            } else {
                Vertex vtxNext = faceVtxs;
                for (Vertex vtx = vtxNext; vtx != null; vtx = vtxNext) {
                    vtxNext = vtx.next;
                    double dist = absorbingFace.distanceToPlane(vtx.pnt);
                    if (dist > tolerance) {
                        addPointToFace(vtx, absorbingFace);
                    } else {
                        unclaimed.add(vtx);
                    }
                }
            }
        }
    }

    private Vertex removeAllPointsFromFace(Face face) {
        if (face.outside != null) {
            Vertex end = face.outside;
            while (end.next != null && end.next.face == face) {
                end = end.next;
            }
            claimed.delete(face.outside, end);
            end.next = null;
            return face.outside;
        } else {
            return null;
        }
    }

    private void createInitialSimplex() {
        double max = 0;
        int imax = 0;

        for (int i = 0; i < 3; i++) {
            double diff = maxVtxs[i].getPnt().get(i) - minVtxs[i].getPnt().get(i);
            if (diff > max) {
                max = diff;
                imax = i;
            }
        }

        if (max <= tolerance) {
            throw new IllegalArgumentException("Input points appear to be coincident");
        }
        QuickHullVertex[] vtx = new QuickHullVertex[4];
        // set first two vertices to be those with the greatest
        // one dimensional separation

        vtx[0] = maxVtxs[imax];
        vtx[1] = minVtxs[imax];

        // set third vertex to be the vertex farthest from
        // the line between vtx0 and vtx1
        Vector3f diff02 = new Vector3f();
        Vector3f nrml = new Vector3f();
        Vector3f xprod = new Vector3f();
        double maxSqr = 0;
        Vector3f u01=vtx[1].getPnt().sub(vtx[0].getPnt()).normalize();
        for (int i = 0; i < numPoints; i++) {
            diff02.sub(pointBuffer[i].pnt, vtx[0].getPnt());
            xprod.cross(u01, diff02);
            double lenSqr = xprod.normSquared();
            if (lenSqr > maxSqr && pointBuffer[i] != vtx[0] && // paranoid
                    pointBuffer[i] != vtx[1]) {
                maxSqr = lenSqr;
                vtx[2] = pointBuffer[i];
                nrml.set(xprod);
            }
        }
        if (Math.sqrt(maxSqr) <= 100 * tolerance) {
            throw new IllegalArgumentException("Input points appear to be colinear");
        }
        nrml.normalize();

        double maxDist = 0;
        double d0 = vtx[2].getPnt().dot(nrml);
        for (int i = 0; i < numPoints; i++) {
            double dist = Math.abs(pointBuffer[i].pnt.dot(nrml) - d0);
            if (dist > maxDist && pointBuffer[i] != vtx[0] && // paranoid
                    pointBuffer[i] != vtx[1] && pointBuffer[i] != vtx[2]) {
                maxDist = dist;
                vtx[3] = pointBuffer[i];
            }
        }
        if (Math.abs(maxDist) <= 100 * tolerance) {
            throw new IllegalArgumentException("Input points appear to be coplanar");
        }

        QuickHullFace[] tris = new QuickHullFace[4];

        if (vtx[3].getPnt().dot(nrml) - d0 < 0) {
            tris[0] = Face.createTriangle(vtx[0], vtx[1], vtx[2]);
            tris[1] = Face.createTriangle(vtx[3], vtx[1], vtx[0]);
            tris[2] = Face.createTriangle(vtx[3], vtx[2], vtx[1]);
            tris[3] = Face.createTriangle(vtx[3], vtx[0], vtx[2]);

            for (int i = 0; i < 3; i++) {
                int k = (i + 1) % 3;
                tris[i + 1].getEdge(1).setOpposite(tris[k + 1].getEdge(0));
                tris[i + 1].getEdge(2).setOpposite(tris[0].getEdge(k));
            }
        } else {
            tris[0] = Face.createTriangle(vtx[0], vtx[2], vtx[1]);
            tris[1] = Face.createTriangle(vtx[3], vtx[0], vtx[1]);
            tris[2] = Face.createTriangle(vtx[3], vtx[1], vtx[2]);
            tris[3] = Face.createTriangle(vtx[3], vtx[2], vtx[0]);

            for (int i = 0; i < 3; i++) {
                int k = (i + 1) % 3;
                tris[i + 1].getEdge(0).setOpposite(tris[k + 1].getEdge(1));
                tris[i + 1].getEdge(2).setOpposite(tris[0].getEdge((3 - i) % 3));
            }
        }

        for (int i = 0; i < 4; i++) {
            faces.add(tris[i]);
        }

        for (int i = 0; i < numPoints; i++) {
            QuickHullVertex v = pointBuffer[i];

            if (v == vtx[0] || v == vtx[1] || v == vtx[2] || v == vtx[3]) {
                continue;
            }

            maxDist = tolerance;
            QuickHullFace maxFace = null;
            for (int k = 0; k < 4; k++) {
                double dist = tris[k].distanceToPlane(v.pnt);
                if (dist > maxDist) {
                    maxFace = tris[k];
                    maxDist = dist;
                }
            }
            if (maxFace != null) {
                addPointToFace(v, maxFace);
            }
        }
    }

    private void addPointToFace(QuickHullVertex vtx, QuickHullFace face) {
        vtx.face = face;

        if (face.outside == null) {
            claimed.add(vtx);
        } else {
            claimed.insertBefore(vtx, face.outside);
        }
        face.outside = vtx;
    }

    private void computeMaxAndMin() {
        Vector3f max = new Vector3f();
        Vector3f min = new Vector3f();

        for (int i = 0; i < 3; i++) {
            maxVtxs[i] = minVtxs[i] = hullVertices.get(0);
        }
        max.set(hullVertices.get(0).getPnt());
        min.set(hullVertices.get(0).getPnt());

        for (int i = 1; i < hullVertices.size(); i++) {
            Vector3f pnt = hullVertices.get(i).getPnt();
            if (pnt.x > max.x) {
                max.x = pnt.x;
                maxVtxs[0] = hullVertices.get(i);
            } else if (pnt.x < min.x) {
                min.x = pnt.x;
                minVtxs[0] = hullVertices.get(i);
            }
            if (pnt.y > max.y) {
                max.y = pnt.y;
                maxVtxs[1] = hullVertices.get(i);
            } else if (pnt.y < min.y) {
                min.y = pnt.y;
                minVtxs[1] = hullVertices.get(i);
            }
            if (pnt.z > max.z) {
                max.z = pnt.z;
                maxVtxs[2] = hullVertices.get(i);
            } else if (pnt.z < min.z) {
                min.z = pnt.z;
                minVtxs[2] = hullVertices.get(i);
            }
        }

        maxAxisLength = Math.max(max.x - min.x, max.y - min.y);
        maxAxisLength = Math.max(max.z - min.z, maxAxisLength);

        tolerance =
                3 * DOUBLE_PREC * (Math.max(Math.abs(max.x), Math.abs(min.x)) + Math.max(Math.abs(max.y), Math.abs(min.y)) + Math.max(Math.abs(max.z), Math.abs(min.z)));

    }

}
