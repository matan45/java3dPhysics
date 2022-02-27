package collision.broadphase;

import body.CollisionBody;
import collision.shapes.AABB;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class SweepAndPruneAlgorithm extends BroadPhaseAlgorithm {
    // Invalid array index
    private static final int INVALID_INDEX = Integer.MAX_VALUE;

    // Number of sentinel end-points in the array of a given axis
    private static final int NB_SENTINELS = 2;

    // Array that contains all the AABB boxes of the broad-phase
    private BoxAABB[] mBoxes = null;

    // Array of end-points on the three axis
    private final EndPoint[][] mEndPoints = {null, null, null};

    // Number of AABB boxes in the broad-phase
    private int mNbBoxes = 0;

    // Max number of boxes in the boxes array
    private int mNbMaxBoxes = 0;

    // Indices that are not used by any boxes
    protected final List<Integer> mFreeBoxIndices = new ArrayList<>();

    // Map a body pointer to a box index
    protected final Map<CollisionBody, Integer> mMapBodyToBoxIndex = new HashMap<>();

    @Override
    public void addObject(CollisionBody body, AABB aabb) {

    }

    @Override
    public void removeObject(CollisionBody body) {

    }

    @Override
    public void updateObject(CollisionBody body, AABB aabb) {

    }
}
