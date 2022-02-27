package collision.broadphase;

import body.CollisionBody;
import collision.shapes.AABB;

public abstract class BroadPhaseAlgorithm {

    // Notify the broad-phase about a new object in the world
    public abstract void addObject(CollisionBody body, AABB aabb);

    // Notify the broad-phase about an object that has been removed from the world
    public abstract void removeObject(CollisionBody body);

    // Notify the broad-phase that the AABB of an object has changed
    public abstract void updateObject(CollisionBody body, AABB aabb);
}
