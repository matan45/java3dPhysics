package collisionDetection.narrowPhase.collisionResult;

import math.Vector3f;

import java.util.ArrayList;

public class RayCastResult {
    private Vector3f point;
    private Vector3f normal;
    private float t;
    private boolean hit;

    public RayCastResult() {
        this.hit = false;
        this.point = new Vector3f();
        this.normal = new Vector3f();
        this.t = 0;
    }

    public RayCastResult(Vector3f point, Vector3f normal, float t, boolean hit) {
        this.point = point;
        this.normal = normal;
        this.t = t;
        this.hit = hit;
    }

    public Vector3f getPoint() {
        return point;
    }

    public void setPoint(Vector3f point) {
        this.point = point;
    }

    public Vector3f getNormal() {
        return normal;
    }

    public void setNormal(Vector3f normal) {
        this.normal = normal;
    }

    public float getT() {
        return t;
    }

    public void setT(float t) {
        this.t = t;
    }

    public boolean isHit() {
        return hit;
    }

    public void setHit(boolean hit) {
        this.hit = hit;
    }

    @Override
    public String toString() {
        return "RayCastResult{" +
                "point=" + point +
                ", normal=" + normal +
                ", t=" + t +
                ", hit=" + hit +
                '}';
    }
}
