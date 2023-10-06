package collisionDetection.util.quickHull;

import math.Vector3f;

import java.util.ArrayList;
import java.util.List;

public class QuickHull3D {

    private static final double DOUBLE_PREC = 2.2204460492503131e-16;
    private List<QuickHullVertex> hullVertices;
    private QuickHullVertex[] maxVtxs = new QuickHullVertex[3];
    private QuickHullVertex[] minVtxs = new QuickHullVertex[3];
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
