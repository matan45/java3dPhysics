package collisionDetection.util.quickHull;

import math.Vector3f;

import java.util.ArrayList;
import java.util.List;

public class QuickHull3D {

    public static final double AUTOMATIC_TOLERANCE = -1;
    private static final double DOUBLE_PREC = 2.2204460492503131e-16;
    private double explicitTolerance = AUTOMATIC_TOLERANCE;

    private List<QuickHullVertex> hullVertices;
    private QuickHullVertex[] maxVtxs = new QuickHullVertex[3];
    private QuickHullVertex[] minVtxs = new QuickHullVertex[3];
    // estimated size of the point set
   private double charLength;

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
        int cnt = 0;
        QuickHullVertex eyeVtx;

        computeMaxAndMin();
        createInitialSimplex();
        while ((eyeVtx = nextPointToAdd()) != null) {
            addPointToHull(eyeVtx);
            cnt++;
        }
        reindexFacesAndVertices();
    }

    private QuickHullVertex nextPointToAdd() {
        return null;
    }

    private void reindexFacesAndVertices() {
    }

    private void addPointToHull(QuickHullVertex eyeVtx) {
    }

    private void createInitialSimplex() {
    }

    private void computeMaxAndMin() {
    }

}
