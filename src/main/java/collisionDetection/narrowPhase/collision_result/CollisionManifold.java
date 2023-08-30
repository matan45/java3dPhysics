package collisionDetection.narrowPhase.collision_result;

import math.Vector3f;

import java.util.List;

public class CollisionManifold {
    private boolean colliding;
    private Vector3f normal;
    private float depth;
    private List<Vector3f> contacts;

    public CollisionManifold(boolean colliding, Vector3f normal, float depth, List<Vector3f> contacts) {
        this.colliding = colliding;
        this.normal = normal;
        this.depth = depth;
        this.contacts = contacts;
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

    public List<Vector3f> getContacts() {
        return contacts;
    }

    public void setContacts(List<Vector3f> contacts) {
        this.contacts = contacts;
    }

    @Override
    public String toString() {
        return "CollisionManifold{" +
                "colliding=" + colliding +
                ", normal=" + normal +
                ", depth=" + depth +
                ", contacts=" + contacts +
                '}';
    }
}
