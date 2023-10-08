package collisionDetection.util.quickHull;

import math.Vector3f;

import java.io.PrintStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;

public class QuickHull3D {

    private static final int NONCONVEX_WRT_LARGER_FACE = 1;
    private static final int NONCONVEX = 2;

    private static final double DOUBLE_PREC = 2.2204460492503131e-16;
    protected double charLength;
    protected Vertex[] pointBuffer;
    protected int[] vertexPointIndices;
    private final Face[] discardedFaces;
    private final Vertex[] maxVtxs;
    private final Vertex[] minVtxs;
    protected List<Face> faces;
    protected List<HalfEdge> horizon;
    private final FaceList newFaces;
    private final VertexList unclaimed;
    private final VertexList claimed;
    protected int numVertices;
    protected int numFaces;
    protected int numPoints;
    protected double tolerance;

    public QuickHull3D() {
        this.pointBuffer = new Vertex[0];
        this.vertexPointIndices = new int[0];
        this.discardedFaces = new Face[3];
        this.maxVtxs = new Vertex[3];
        this.minVtxs = new Vertex[3];
        this.faces = new ArrayList<>(16);
        this.horizon = new ArrayList<>(16);
        this.newFaces = new FaceList();
        this.unclaimed = new VertexList();
        this.claimed = new VertexList();
    }

    private void addPointToFace(Vertex vtx, Face face) {
        vtx.face = face;

        if (face.outside == null) {
            claimed.add(vtx);
        } else {
            claimed.insertBefore(vtx, face.outside);
        }
        face.outside = vtx;
    }

    private void removePointFromFace(Vertex vtx, Face face) {
        if (vtx == face.outside) {
            if (vtx.next != null && vtx.next.face == face) {
                face.outside = vtx.next;
            } else {
                face.outside = null;
            }
        }
        claimed.delete(vtx);
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

    public void build(Vector3f[] points) throws IllegalArgumentException {
        build(points, points.length);
    }


    public void build(Vector3f[] points, int nump) throws IllegalArgumentException {
        if (nump < 4) {
            throw new IllegalArgumentException("Less than four input points specified");
        }
        if (points.length < nump) {
            throw new IllegalArgumentException("Point array too small for specified number of points");
        }
        initBuffers(nump);
        setPoints(points, nump);
        buildHull();
    }

    protected void initBuffers(int nump) {
        if (pointBuffer.length < nump) {
            Vertex[] newBuffer = new Vertex[nump];
            vertexPointIndices = new int[nump];

            System.arraycopy(pointBuffer, 0, newBuffer, 0, pointBuffer.length);
            for (int i = pointBuffer.length; i < nump; i++) {
                newBuffer[i] = new Vertex();
            }
            pointBuffer = newBuffer;
        }
        faces.clear();
        claimed.clear();
        numFaces = 0;
        numPoints = nump;
    }

    protected void setPoints(Vector3f[] pnts, int nump) {
        for (int i = 0; i < nump; i++) {
            Vertex vtx = pointBuffer[i];
            vtx.pnt.set(pnts[i]);
            vtx.index = i;
        }
    }

    protected void computeMaxAndMin() {
        Vector3f max = new Vector3f();
        Vector3f min = new Vector3f();

        for (int i = 0; i < 3; i++) {
            maxVtxs[i] = minVtxs[i] = pointBuffer[0];
        }
        max.set(pointBuffer[0].pnt);
        min.set(pointBuffer[0].pnt);

        for (int i = 1; i < numPoints; i++) {
            Vector3f pnt = pointBuffer[i].pnt;
            if (pnt.x > max.x) {
                max.x = pnt.x;
                maxVtxs[0] = pointBuffer[i];
            } else if (pnt.x < min.x) {
                min.x = pnt.x;
                minVtxs[0] = pointBuffer[i];
            }
            if (pnt.y > max.y) {
                max.y = pnt.y;
                maxVtxs[1] = pointBuffer[i];
            } else if (pnt.y < min.y) {
                min.y = pnt.y;
                minVtxs[1] = pointBuffer[i];
            }
            if (pnt.z > max.z) {
                max.z = pnt.z;
                maxVtxs[2] = pointBuffer[i];
            } else if (pnt.z < min.z) {
                min.z = pnt.z;
                minVtxs[2] = pointBuffer[i];
            }
        }

        charLength = Math.max(max.x - min.x, max.y - min.y);
        charLength = Math.max(max.z - min.z, charLength);

        tolerance =
                3 * DOUBLE_PREC * (Math.max(Math.abs(max.x), Math.abs(min.x)) + Math.max(Math.abs(max.y), Math.abs(min.y)) + Math.max(Math.abs(max.z), Math.abs(min.z)));

    }

    private void createInitialSimplex() throws IllegalArgumentException {
        double max = 0;
        int imax = 0;

        for (int i = 0; i < 3; i++) {
            double diff = maxVtxs[i].pnt.get(i) - minVtxs[i].pnt.get(i);
            if (diff > max) {
                max = diff;
                imax = i;
            }
        }

        if (max <= tolerance) {
            throw new IllegalArgumentException("Input points appear to be coincident");
        }
        Vertex[] vtx = new Vertex[4];

        vtx[0] = maxVtxs[imax];
        vtx[1] = minVtxs[imax];

        Vector3f u01 = new Vector3f();
        Vector3f diff02 = new Vector3f();
        Vector3f nrml = new Vector3f();
        Vector3f xprod = new Vector3f();
        double maxSqr = 0;
        u01 = vtx[1].pnt.sub(vtx[0].pnt).normalize();
        for (int i = 0; i < numPoints; i++) {
            diff02.set(pointBuffer[i].pnt.sub(vtx[0].pnt));
            xprod.set(u01.cross(diff02));
            double lenSqr = xprod.lengthSquared();
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
        nrml.set(nrml.normalize());
        double maxDist = 0;
        double d0 = vtx[2].pnt.dot(nrml);
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

        Face[] tris = new Face[4];

        if (vtx[3].pnt.dot(nrml) - d0 < 0) {
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

        faces.addAll(Arrays.asList(tris).subList(0, 4));

        for (int i = 0; i < numPoints; i++) {
            Vertex v = pointBuffer[i];

            if (v == vtx[0] || v == vtx[1] || v == vtx[2] || v == vtx[3]) {
                continue;
            }

            maxDist = tolerance;
            Face maxFace = null;
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

    public Vector3f[] getVertices() {
        Vector3f[] vtxs = new Vector3f[numVertices];
        for (int i = 0; i < numVertices; i++) {
            vtxs[i] = pointBuffer[vertexPointIndices[i]].pnt;
        }
        return vtxs;
    }


    public int getVertices(double[] coords) {
        for (int i = 0; i < numVertices; i++) {
            Vector3f pnt = pointBuffer[vertexPointIndices[i]].pnt;
            coords[i * 3] = pnt.x;
            coords[i * 3 + 1] = pnt.y;
            coords[i * 3 + 2] = pnt.z;
        }
        return numVertices;
    }

    private void resolveUnclaimedPoints(FaceList newFaces) {
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
            }
        }
    }

    private void deleteFacePoints(Face face, Face absorbingFace) {
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


    private double oppFaceDistance(HalfEdge he) {
        return he.face.distanceToPlane(he.opposite.face.getCentroid());
    }

    private boolean doAdjacentMerge(Face face, int mergeType) {
        HalfEdge hedge = face.he0;
        boolean convex = true;
        do {
            Face oppFace = hedge.oppositeFace();
            boolean merge = false;

            if (mergeType == NONCONVEX) {
                if (oppFaceDistance(hedge) > -tolerance || oppFaceDistance(hedge.opposite) > -tolerance) {
                    merge = true;
                }
            } else {
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

    private void calculateHorizon(Vector3f eyePnt, HalfEdge edge0, Face face, List<HalfEdge> horizon) {
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

    private HalfEdge addAdjoiningFace(Vertex eyeVtx, HalfEdge he) {
        Face face = Face.createTriangle(eyeVtx, he.tail(), he.head());
        faces.add(face);
        face.getEdge(-1).setOpposite(he.getOpposite());
        return face.getEdge(0);
    }

    private void addNewFaces(FaceList newFaces, Vertex eyeVtx, List<HalfEdge> horizon) {
        newFaces.clear();

        HalfEdge hedgeSidePrev = null;
        HalfEdge hedgeSideBegin = null;

        for (HalfEdge horizonHe : horizon) {
            HalfEdge hedgeSide = addAdjoiningFace(eyeVtx, horizonHe);

            if (hedgeSidePrev != null) {
                hedgeSide.next.setOpposite(hedgeSidePrev);
            } else {
                hedgeSideBegin = hedgeSide;
            }
            newFaces.add(hedgeSide.getFace());
            hedgeSidePrev = hedgeSide;
        }
        assert hedgeSideBegin != null;
        hedgeSideBegin.next.setOpposite(hedgeSidePrev);
    }

    private Vertex nextPointToAdd() {
        if (!claimed.isEmpty()) {
            Face eyeFace = claimed.first().face;
            Vertex eyeVtx = null;
            double maxDist = 0;
            for (Vertex vtx = eyeFace.outside; vtx != null && vtx.face == eyeFace; vtx = vtx.next) {
                double dist = eyeFace.distanceToPlane(vtx.pnt);
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

    private void addPointToHull(Vertex eyeVtx) {
        horizon.clear();
        unclaimed.clear();

        removePointFromFace(eyeVtx, eyeVtx.face);
        calculateHorizon(eyeVtx.pnt, null, eyeVtx.face, horizon);
        newFaces.clear();
        addNewFaces(newFaces, eyeVtx, horizon);

        for (Face face = newFaces.first(); face != null; face = face.next) {
            if (face.mark == Face.VISIBLE) {
                while (doAdjacentMerge(face, NONCONVEX_WRT_LARGER_FACE)) ;
            }
        }

        for (Face face = newFaces.first(); face != null; face = face.next) {
            if (face.mark == Face.NON_CONVEX) {
                face.mark = Face.VISIBLE;
                while (doAdjacentMerge(face, NONCONVEX)) ;
            }
        }
        resolveUnclaimedPoints(newFaces);
    }

    protected void buildHull() {
        Vertex eyeVtx;

        computeMaxAndMin();
        createInitialSimplex();
        while ((eyeVtx = nextPointToAdd()) != null) {
            addPointToHull(eyeVtx);
        }
        reindexFacesAndVertices();
    }

    private void markFaceVertices(Face face, int mark) {
        HalfEdge he0 = face.getFirstEdge();
        HalfEdge he = he0;
        do {
            he.head().index = mark;
            he = he.next;
        } while (he != he0);
    }

    private void reindexFacesAndVertices() {
        for (int i = 0; i < numPoints; i++) {
            pointBuffer[i].index = -1;
        }

        numFaces = 0;
        for (Iterator<Face> it = faces.iterator(); it.hasNext(); ) {
            Face face = it.next();
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
}
