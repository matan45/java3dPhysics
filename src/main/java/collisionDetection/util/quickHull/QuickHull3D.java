package collisionDetection.util.quickHull;

import math.Vector3f;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

public class QuickHull3D {

    private static final double DOUBLE_PREC = 2.2204460492503131e-16;
    private static final int NONCONVEX_WRT_LARGER_FACE = 1;
    private static final int NONCONVEX = 2;

    private List<QuickHullVertex> pointBuffer;
    private int numPoints;
    int numFaces;
    int numVertices;
    private QuickHullVertex[] maxVtxs = new QuickHullVertex[3];
    int[] vertexPointIndices;
    private QuickHullVertex[] minVtxs = new QuickHullVertex[3];
    private QuickHullVertexList unclaimed = new QuickHullVertexList();
    private QuickHullVertexList claimed = new QuickHullVertexList();
    private QuickHullFace[] discardedFaces = new QuickHullFace[3];
    private List<QuickHullHalfEdge> horizon = new ArrayList<>(16);
    private List<QuickHullFace> faces = new ArrayList<>(16);
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
        pointBuffer = new ArrayList<>(points.size());
        vertexPointIndices = new int[points.size()];
        for (Vector3f point : points) {
            QuickHullVertex quickHullVertex = new QuickHullVertex();
            quickHullVertex.setPnt(point);
            pointBuffer.add(quickHullVertex);
        }
        numPoints = points.size();
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
            pointBuffer.get(i).setIndex(-1);
        }
        // remove inactive faces and mark active vertices
        numFaces = 0;
        for (Iterator<QuickHullFace> it = faces.iterator(); it.hasNext(); ) {
            QuickHullFace face = it.next();
            if (face.mark != QuickHullFace.VISIBLE) {
                it.remove();
            } else {
                markFaceVertices(face, 0);
                numFaces++;
            }
        }
        // reindex vertices
        numVertices = 0;
        for (int i = 0; i < numPoints; i++) {
            QuickHullVertex vtx = pointBuffer.get(i);
            if (vtx.getIndex() == 0) {
                vertexPointIndices[numVertices] = i;
                vtx.setIndex(numVertices++);
            }
        }
    }

    private void markFaceVertices(QuickHullFace face, int mark) {
        QuickHullHalfEdge he0 = face.getFirstEdge();
        QuickHullHalfEdge he = he0;
        do {
            he.getHead().setIndex(mark);
            he = he.getNext();
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

        for (QuickHullFace face = newFaces.first(); face != null; face = face.next) {
            if (face.mark == QuickHullFace.VISIBLE) {
                while (doAdjacentMerge(face, NONCONVEX_WRT_LARGER_FACE)) ;
            }
        }
        // second merge pass ... merge faces which are non-convex
        // wrt either face
        for (QuickHullFace face = newFaces.first(); face != null; face = face.next) {
            if (face.mark == QuickHullFace.NON_CONVEX) {
                face.mark = QuickHullFace.VISIBLE;
                while (doAdjacentMerge(face, NONCONVEX)) ;
            }
        }
        resolveUnclaimedPoints(newFaces);
    }

    protected void resolveUnclaimedPoints(QuickHullFaceList newFaces) {
        QuickHullVertex vtxNext = unclaimed.first();
        for (QuickHullVertex vtx = vtxNext; vtx != null; vtx = vtxNext) {
            vtxNext = vtx.getNext();

            double maxDist = tolerance;
            QuickHullFace maxFace = null;
            for (QuickHullFace newFace = newFaces.first(); newFace != null; newFace = newFace.next) {
                if (newFace.mark == QuickHullFace.VISIBLE) {
                    double dist = newFace.distanceToPlane(vtx.getPnt());
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

            }
        }
    }

    private boolean doAdjacentMerge(QuickHullFace face, int mergeType) {
        QuickHullHalfEdge hedge = face.he0;

        boolean convex = true;
        do {
            QuickHullFace oppFace = hedge.getOppositeFace();
            boolean merge = false;

            if (mergeType == NONCONVEX) { // then merge faces if they are
                // definitively non-convex
                if (oppFaceDistance(hedge) > -tolerance || oppFaceDistance(hedge.getOpposite()) > -tolerance) {
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
                    } else if (oppFaceDistance(hedge.getOpposite()) > -tolerance) {
                        convex = false;
                    }
                } else {
                    if (oppFaceDistance(hedge.getOpposite()) > -tolerance) {
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
            hedge = hedge.getNext();
        } while (hedge != face.he0);
        if (!convex) {
            face.mark = QuickHullFace.NON_CONVEX;
        }
        return false;
    }

    protected double oppFaceDistance(QuickHullHalfEdge he) {
        return he.getFace().distanceToPlane(he.getOpposite().getFace().getCentroid());
    }

    protected void addNewFaces(QuickHullFaceList newFaces, QuickHullVertex eyeVtx, List<QuickHullHalfEdge> horizon) {
        newFaces.clear();

        QuickHullHalfEdge hedgeSidePrev = null;
        QuickHullHalfEdge hedgeSideBegin = null;

        for (QuickHullHalfEdge horizonHe : horizon) {
            QuickHullHalfEdge hedgeSide = addAdjoiningFace(eyeVtx, horizonHe);

            if (hedgeSidePrev != null) {
                hedgeSide.getNext().setOpposite(hedgeSidePrev);
            } else {
                hedgeSideBegin = hedgeSide;
            }
            newFaces.add(hedgeSide.getFace());
            hedgeSidePrev = hedgeSide;
        }
        assert hedgeSideBegin != null;
        hedgeSideBegin.getNext().setOpposite(hedgeSidePrev);
    }

    private QuickHullHalfEdge addAdjoiningFace(QuickHullVertex eyeVtx, QuickHullHalfEdge he) {
        QuickHullFace face = QuickHullFace.createTriangle(eyeVtx, he.getTail(), he.getHead());
        faces.add(face);
        face.getEdge(-1).setOpposite(he.getOpposite());
        return face.getEdge(0);
    }

    protected void calculateHorizon(Vector3f eyePnt, QuickHullHalfEdge edge0, QuickHullFace face, List<QuickHullHalfEdge> horizon) {
        // oldFaces.add (face);
        deleteFacePoints(face, null);
        face.mark = QuickHullFace.DELETED;

        QuickHullHalfEdge edge;
        if (edge0 == null) {
            edge0 = face.getEdge(0);
            edge = edge0;
        } else {
            edge = edge0.getNext();
        }
        do {
            QuickHullFace oppFace = edge.getOppositeFace();
            if (oppFace.mark == QuickHullFace.VISIBLE) {
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
            if (vtx.getNext() != null && vtx.getNext().getFace() == face) {
                face.outside = vtx.getNext();
            } else {
                face.outside = null;
            }
        }
        claimed.delete(vtx);
    }

    protected void deleteFacePoints(QuickHullFace face, QuickHullFace absorbingFace) {
        QuickHullVertex faceVtxs = removeAllPointsFromFace(face);
        if (faceVtxs != null) {
            if (absorbingFace == null) {
                unclaimed.addAll(faceVtxs);
            } else {
                QuickHullVertex vtxNext = faceVtxs;
                for (QuickHullVertex vtx = vtxNext; vtx != null; vtx = vtxNext) {
                    vtxNext = vtx.getNext();
                    double dist = absorbingFace.distanceToPlane(vtx.getPnt());
                    if (dist > tolerance) {
                        addPointToFace(vtx, absorbingFace);
                    } else {
                        unclaimed.add(vtx);
                    }
                }
            }
        }
    }

    private QuickHullVertex removeAllPointsFromFace(QuickHullFace face) {
        if (face.outside != null) {
            QuickHullVertex end = face.outside;
            while (end.getNext() != null && end.getNext().getFace() == face) {
                end = end.getNext();
            }
            claimed.delete(face.outside, end);
            end.setNext(null);
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
        Vector3f u01 = vtx[1].getPnt().sub(vtx[0].getPnt()).normalize();
        for (int i = 0; i < numPoints; i++) {
            diff02 = pointBuffer.get(i).getPnt().sub(vtx[0].getPnt());
            xprod = u01.cross(diff02);
            double lenSqr = xprod.lengthSquared();
            if (lenSqr > maxSqr && pointBuffer.get(i) != vtx[0] && // paranoid
                    pointBuffer.get(i) != vtx[1]) {
                maxSqr = lenSqr;
                vtx[2] = pointBuffer.get(i);
                nrml.set(xprod);
            }
        }
        if (Math.sqrt(maxSqr) <= 100 * tolerance) {
            throw new IllegalArgumentException("Input points appear to be colinear");
        }
        nrml.set(nrml.normalize());
        double maxDist = 0;
        double d0 = vtx[2].getPnt().dot(nrml);
        for (int i = 0; i < numPoints; i++) {
            double dist = Math.abs(pointBuffer.get(i).getPnt().dot(nrml) - d0);
            if (dist > maxDist && pointBuffer.get(i) != vtx[0] && // paranoid
                    pointBuffer.get(i) != vtx[1] && pointBuffer.get(i) != vtx[2]) {
                maxDist = dist;
                vtx[3] = pointBuffer.get(i);
            }
        }
        if (Math.abs(maxDist) <= 100 * tolerance) {
            throw new IllegalArgumentException("Input points appear to be coplanar");
        }

        QuickHullFace[] tris = new QuickHullFace[4];

        if (vtx[3].getPnt().dot(nrml) - d0 < 0) {
            tris[0] = QuickHullFace.createTriangle(vtx[0], vtx[1], vtx[2]);
            tris[1] = QuickHullFace.createTriangle(vtx[3], vtx[1], vtx[0]);
            tris[2] = QuickHullFace.createTriangle(vtx[3], vtx[2], vtx[1]);
            tris[3] = QuickHullFace.createTriangle(vtx[3], vtx[0], vtx[2]);

            for (int i = 0; i < 3; i++) {
                int k = (i + 1) % 3;
                tris[i + 1].getEdge(1).setOpposite(tris[k + 1].getEdge(0));
                tris[i + 1].getEdge(2).setOpposite(tris[0].getEdge(k));
            }
        } else {
            tris[0] = QuickHullFace.createTriangle(vtx[0], vtx[2], vtx[1]);
            tris[1] = QuickHullFace.createTriangle(vtx[3], vtx[0], vtx[1]);
            tris[2] = QuickHullFace.createTriangle(vtx[3], vtx[1], vtx[2]);
            tris[3] = QuickHullFace.createTriangle(vtx[3], vtx[2], vtx[0]);

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
            QuickHullVertex v = pointBuffer.get(i);

            if (v == vtx[0] || v == vtx[1] || v == vtx[2] || v == vtx[3]) {
                continue;
            }

            maxDist = tolerance;
            QuickHullFace maxFace = null;
            for (int k = 0; k < 4; k++) {
                double dist = tris[k].distanceToPlane(v.getPnt());
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
        vtx.setFace(face);

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
            maxVtxs[i] = minVtxs[i] = pointBuffer.get(0);
        }
        max.set(pointBuffer.get(0).getPnt());
        min.set(pointBuffer.get(0).getPnt());

        for (int i = 1; i < pointBuffer.size(); i++) {
            Vector3f pnt = pointBuffer.get(i).getPnt();
            if (pnt.x > max.x) {
                max.x = pnt.x;
                maxVtxs[0] = pointBuffer.get(i);
            } else if (pnt.x < min.x) {
                min.x = pnt.x;
                minVtxs[0] = pointBuffer.get(i);
            }
            if (pnt.y > max.y) {
                max.y = pnt.y;
                maxVtxs[1] = pointBuffer.get(i);
            } else if (pnt.y < min.y) {
                min.y = pnt.y;
                minVtxs[1] = pointBuffer.get(i);
            }
            if (pnt.z > max.z) {
                max.z = pnt.z;
                maxVtxs[2] = pointBuffer.get(i);
            } else if (pnt.z < min.z) {
                min.z = pnt.z;
                minVtxs[2] = pointBuffer.get(i);
            }
        }

        maxAxisLength = Math.max(max.x - min.x, max.y - min.y);
        maxAxisLength = Math.max(max.z - min.z, maxAxisLength);

        tolerance =
                3 * DOUBLE_PREC * (Math.max(Math.abs(max.x), Math.abs(min.x)) + Math.max(Math.abs(max.y), Math.abs(min.y)) + Math.max(Math.abs(max.z), Math.abs(min.z)));

    }

}
