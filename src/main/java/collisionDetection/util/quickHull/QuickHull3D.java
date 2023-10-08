package collisionDetection.util.quickHull;

import math.Vector3f;

import java.io.IOException;
import java.io.InputStream;
import java.io.PrintStream;
import java.util.Iterator;
import java.util.Vector;

public class QuickHull3D {

    public static final int CLOCKWISE = 0x1;

    public static final int INDEXED_FROM_ONE = 0x2;

    public static final int INDEXED_FROM_ZERO = 0x4;

    public static final int POINT_RELATIVE = 0x8;

    public static final double AUTOMATIC_TOLERANCE = -1;

    protected int findIndex = -1;

    protected double charLength;

    protected Vertex[] pointBuffer = new Vertex[0];

    protected int[] vertexPointIndices = new int[0];

    private Face[] discardedFaces = new Face[3];

    private Vertex[] maxVtxs = new Vertex[3];

    private Vertex[] minVtxs = new Vertex[3];

    protected Vector faces = new Vector(16);

    protected Vector horizon = new Vector(16);

    private FaceList newFaces = new FaceList();

    private VertexList unclaimed = new VertexList();

    private VertexList claimed = new VertexList();

    protected int numVertices;

    protected int numFaces;

    protected int numPoints;

    protected double explicitTolerance = AUTOMATIC_TOLERANCE;

    protected double tolerance;

    private static final double DOUBLE_PREC = 2.2204460492503131e-16;

    public double getDistanceTolerance() {
        return tolerance;
    }

    public void setExplicitDistanceTolerance(double tol) {
        explicitTolerance = tol;
    }

    public double getExplicitDistanceTolerance() {
        return explicitTolerance;
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

    public QuickHull3D() {
    }

    public QuickHull3D(double[] coords) throws IllegalArgumentException {
        build(coords, coords.length / 3);
    }


    public QuickHull3D(Vector3f[] points) throws IllegalArgumentException {
        build(points, points.length);
    }

    private HalfEdge findHalfEdge(Vertex tail, Vertex head) {
        // brute force ... OK, since setHull is not used much
        for (Iterator it = faces.iterator(); it.hasNext();) {
            HalfEdge he = ((Face) it.next()).findEdge(tail, head);
            if (he != null) {
                return he;
            }
        }
        return null;
    }

    protected void setHull(double[] coords, int nump, int[][] faceIndices, int numf) {
        initBuffers(nump);
        setPoints(coords, nump);
        computeMaxAndMin();
        for (int i = 0; i < numf; i++) {
            Face face = Face.create(pointBuffer, faceIndices[i]);
            HalfEdge he = face.he0;
            do {
                HalfEdge heOpp = findHalfEdge(he.head(), he.tail());
                if (heOpp != null) {
                    he.setOpposite(heOpp);
                }
                he = he.next;
            } while (he != face.he0);
            faces.add(face);
        }
    }

    private void printQhullErrors(Process proc) throws IOException {
        boolean wrote = false;
        InputStream es = proc.getErrorStream();
        StringBuffer error = new StringBuffer();
        while (es.available() > 0) {
            error.append((char) es.read());
            wrote = true;
        }
        if (wrote) {
            error.append(" ");
        }
    }


    public void printPoints(PrintStream ps) {
        for (int i = 0; i < numPoints; i++) {
            Vector3f pnt = pointBuffer[i].pnt;
            ps.println(pnt.x + ", " + pnt.y + ", " + pnt.z + ",");
        }
    }


    public void build(double[] coords) throws IllegalArgumentException {
        build(coords, coords.length / 3);
    }


    public void build(double[] coords, int nump) throws IllegalArgumentException {
        if (nump < 4) {
            throw new IllegalArgumentException("Less than four input points specified");
        }
        if (coords.length / 3 < nump) {
            throw new IllegalArgumentException("Coordinate array too small for specified number of points");
        }
        initBuffers(nump);
        setPoints(coords, nump);
        buildHull();
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

    public void triangulate() {
        double minArea = 1000 * charLength * DOUBLE_PREC;
        newFaces.clear();
        for (Iterator it = faces.iterator(); it.hasNext();) {
            Face face = (Face) it.next();
            if (face.mark == Face.VISIBLE) {
                face.triangulate(newFaces, minArea);
                // splitFace (face);
            }
        }
        for (Face face = newFaces.first(); face != null; face = face.next) {
            faces.add(face);
        }
    }

    protected void initBuffers(int nump) {
        if (pointBuffer.length < nump) {
            Vertex[] newBuffer = new Vertex[nump];
            vertexPointIndices = new int[nump];
            for (int i = 0; i < pointBuffer.length; i++) {
                newBuffer[i] = pointBuffer[i];
            }
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

    protected void setPoints(double[] coords, int nump) {
        for (int i = 0; i < nump; i++) {
            Vertex vtx = pointBuffer[i];
            vtx.pnt.setXYZ((float) coords[i * 3 + 0], (float) coords[i * 3 + 1], (float) coords[i * 3 + 2]);
            vtx.index = i;
        }
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

        // this epsilon formula comes from QuickHull, and I'm
        // not about to quibble.
        charLength = Math.max(max.x - min.x, max.y - min.y);
        charLength = Math.max(max.z - min.z, charLength);
        if (explicitTolerance == AUTOMATIC_TOLERANCE) {
            tolerance =
                    3 * DOUBLE_PREC * (Math.max(Math.abs(max.x), Math.abs(min.x)) + Math.max(Math.abs(max.y), Math.abs(min.y)) + Math.max(Math.abs(max.z), Math.abs(min.z)));
        } else {
            tolerance = explicitTolerance;
        }
    }

    protected void createInitialSimplex() throws IllegalArgumentException {
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
        // set first two vertices to be those with the greatest
        // one dimensional separation

        vtx[0] = maxVtxs[imax];
        vtx[1] = minVtxs[imax];

        // set third vertex to be the vertex farthest from
        // the line between vtx0 and vtx1
        Vector3f u01 = new Vector3f();
        Vector3f diff02 = new Vector3f();
        Vector3f nrml = new Vector3f();
        Vector3f xprod = new Vector3f();
        double maxSqr = 0;
        u01=vtx[1].pnt.sub(vtx[0].pnt).normalize();
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

        for (int i = 0; i < 4; i++) {
            faces.add(tris[i]);
        }

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

    public int getNumVertices() {
        return numVertices;
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
            coords[i * 3 + 0] = pnt.x;
            coords[i * 3 + 1] = pnt.y;
            coords[i * 3 + 2] = pnt.z;
        }
        return numVertices;
    }


    public int[] getVertexPointIndices() {
        int[] indices = new int[numVertices];
        for (int i = 0; i < numVertices; i++) {
            indices[i] = vertexPointIndices[i];
        }
        return indices;
    }


    public int getNumFaces() {
        return faces.size();
    }


    public int[][] getFaces() {
        return getFaces(0);
    }

    public int[][] getFaces(int indexFlags) {
        int[][] allFaces = new int[faces.size()][];
        int k = 0;
        for (Iterator it = faces.iterator(); it.hasNext();) {
            Face face = (Face) it.next();
            allFaces[k] = new int[face.numVertices()];
            getFaceIndices(allFaces[k], face, indexFlags);
            k++;
        }
        return allFaces;
    }


    public void print(PrintStream ps) {
        print(ps, 0);
    }


    public void print(PrintStream ps, int indexFlags) {
        if ((indexFlags & INDEXED_FROM_ZERO) == 0) {
            indexFlags |= INDEXED_FROM_ONE;
        }
        for (int i = 0; i < numVertices; i++) {
            Vector3f pnt = pointBuffer[vertexPointIndices[i]].pnt;
            ps.println("v " + pnt.x + " " + pnt.y + " " + pnt.z);
        }
        for (Iterator fi = faces.iterator(); fi.hasNext();) {
            Face face = (Face) fi.next();
            int[] indices = new int[face.numVertices()];
            getFaceIndices(indices, face, indexFlags);

            ps.print("f");
            for (int k = 0; k < indices.length; k++) {
                ps.print(" " + indices[k]);
            }
            ps.println("");
        }
    }

    private void getFaceIndices(int[] indices, Face face, int flags) {
        boolean ccw = (flags & CLOCKWISE) == 0;
        boolean indexedFromOne = (flags & INDEXED_FROM_ONE) != 0;
        boolean pointRelative = (flags & POINT_RELATIVE) != 0;

        HalfEdge hedge = face.he0;
        int k = 0;
        do {
            int idx = hedge.head().index;
            if (pointRelative) {
                idx = vertexPointIndices[idx];
            }
            if (indexedFromOne) {
                idx++;
            }
            indices[k++] = idx;
            hedge = (ccw ? hedge.next : hedge.prev);
        } while (hedge != face.he0);
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
            } else {

            }
        }
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

    private static final int NONCONVEX_WRT_LARGER_FACE = 1;

    private static final int NONCONVEX = 2;

    protected double oppFaceDistance(HalfEdge he) {
        return he.face.distanceToPlane(he.opposite.face.getCentroid());
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

    protected void calculateHorizon(Vector3f eyePnt, HalfEdge edge0, Face face, Vector horizon) {
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

    private HalfEdge addAdjoiningFace(Vertex eyeVtx, HalfEdge he) {
        Face face = Face.createTriangle(eyeVtx, he.tail(), he.head());
        faces.add(face);
        face.getEdge(-1).setOpposite(he.getOpposite());
        return face.getEdge(0);
    }

    protected void addNewFaces(FaceList newFaces, Vertex eyeVtx, Vector horizon) {
        newFaces.clear();

        HalfEdge hedgeSidePrev = null;
        HalfEdge hedgeSideBegin = null;

        for (Iterator it = horizon.iterator(); it.hasNext();) {
            HalfEdge horizonHe = (HalfEdge) it.next();
            HalfEdge hedgeSide = addAdjoiningFace(eyeVtx, horizonHe);

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

    protected Vertex nextPointToAdd() {
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

    protected void addPointToHull(Vertex eyeVtx) {
        horizon.clear();
        unclaimed.clear();

        removePointFromFace(eyeVtx, eyeVtx.face);
        calculateHorizon(eyeVtx.pnt, null, eyeVtx.face, horizon);
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

    protected boolean checkFaceConvexity(Face face, double tol, PrintStream ps) {
        double dist;
        HalfEdge he = face.he0;
        do {
            face.checkConsistency();
            // make sure edge is convex
            dist = oppFaceDistance(he);
            if (dist > tol) {
                if (ps != null) {
                    ps.println("Edge " + he.getVertexString() + " non-convex by " + dist);
                }
                return false;
            }
            dist = oppFaceDistance(he.opposite);
            if (dist > tol) {
                if (ps != null) {
                    ps.println("Opposite edge " + he.opposite.getVertexString() + " non-convex by " + dist);
                }
                return false;
            }
            if (he.next.oppositeFace() == he.oppositeFace()) {
                if (ps != null) {
                    ps.println("Redundant vertex " + he.head().index + " in face " + face.getVertexString());
                }
                return false;
            }
            he = he.next;
        } while (he != face.he0);
        return true;
    }

    protected boolean checkFaces(double tol, PrintStream ps) {
        // check edge convexity
        boolean convex = true;
        for (Iterator it = faces.iterator(); it.hasNext();) {
            Face face = (Face) it.next();
            if (face.mark == Face.VISIBLE && !checkFaceConvexity(face, tol, ps)) {
                convex = false;
            }
        }
        return convex;
    }


    public boolean check(PrintStream ps) {
        return check(ps, getDistanceTolerance());
    }


    public boolean check(PrintStream ps, double tol)
    {
        // check to make sure all edges are fully connected
        // and that the edges are convex
        double dist;
        double pointTol = 10 * tol;

        if (!checkFaces(tolerance, ps)) {
            return false;
        }

        // check point inclusion

        for (int i = 0; i < numPoints; i++) {
            Vector3f pnt = pointBuffer[i].pnt;
            for (Iterator it = faces.iterator(); it.hasNext();) {
                Face face = (Face) it.next();
                if (face.mark == Face.VISIBLE) {
                    dist = face.distanceToPlane(pnt);
                    if (dist > pointTol) {
                        if (ps != null) {
                            ps.println("Point " + i + " " + dist + " above face " + face.getVertexString());
                        }
                        return false;
                    }
                }
            }
        }
        return true;
    }


}
