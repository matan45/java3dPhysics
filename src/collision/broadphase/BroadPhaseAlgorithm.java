package collision.broadphase;

import body.BodyPair;
import body.CollisionBody;
import collision.shapes.AABB;

public abstract class BroadPhaseAlgorithm {
    protected final PairManager mPairManager;

    public BroadPhaseAlgorithm() {
        mPairManager = new PairManager();
    }

    /**
     * Returns the array of overlapping pairs managed by the pair manager, for iteration purposes. Note that the array returned contains trailing null elements.
     *
     * @return The array of overlapping pairs
     */

    public BodyPair[] getOverlappingPairs() {
        return mPairManager.getOverlappingPairs();
    }

    /**
     * Return the last overlapping pair (used to iterate over the overlapping pairs) or returns null if there are no overlapping pairs. Note that the array returned by {@link #getOverlappingPairs()}
     * contains trailing null elements.
     *
     * @return The last overlapping pair
     */
    public BodyPair getLastOverlappingPair() {
        return mPairManager.getLastOverlappingPair();
    }

    // Notify the broad-phase about a new object in the world
    public abstract void addObject(CollisionBody body, AABB aabb);

    // Notify the broad-phase about an object that has been removed from the world
    public abstract void removeObject(CollisionBody body);

    // Notify the broad-phase that the AABB of an object has changed
    public abstract void updateObject(CollisionBody body, AABB aabb);
}
