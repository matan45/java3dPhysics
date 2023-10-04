package collisionDetection.narrowPhase.collisionResult;

import math.Vector3f;

import java.util.ArrayList;
import java.util.List;
//TODO instead of contactPoints do bodyA and bodyB contactPoints
public class CollisionResult {
    private boolean colliding;
    private Vector3f normal;
    private float depth;
    private List<Vector3f> contactPoints;

    public CollisionResult() {
        this.colliding = false;
        this.normal = new Vector3f();
        this.depth = 0;
        this.contactPoints = new ArrayList<>();
    }

    public CollisionResult(boolean colliding, Vector3f normal, float depth, List<Vector3f> contacts) {
        this.colliding = colliding;
        this.normal = normal;
        this.depth = depth;
        this.contactPoints = contacts;
    }

    public boolean isColliding() {
        return colliding;
    }

    public void setColliding(boolean colliding) {
        this.colliding = colliding;
    }

    public Vector3f getNormal() {
        return normal;
    }

    public void setNormal(Vector3f normal) {
        this.normal = normal;
    }

    public float getDepth() {
        return depth;
    }

    public void setDepth(float depth) {
        this.depth = depth;
    }

    public List<Vector3f> getContactPoints() {
        return contactPoints;
    }

    public void setContactPoints(List<Vector3f> contactPoints) {
        this.contactPoints = contactPoints;
    }

    @Override
    public String toString() {
        return "CollisionManifold{" +
                "colliding=" + colliding +
                ", normal=" + normal +
                ", depth=" + depth +
                ", contacts=" + contactPoints +
                '}';
    }
}
