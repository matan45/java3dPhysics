package collision.broadphase;

import body.CollisionBody;
import collision.shapes.AABB;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

public class SweepAndPruneAlgorithm extends BroadPhaseAlgorithm {
    // Invalid array index
    protected static final int INVALID_INDEX = Integer.MAX_VALUE;

    // Number of sentinel end-points in the array of a given axis
    protected static final int NUM_SENTINELS = 2;

    // Number of AABB boxes in the broad-phase
    protected int numBoxes;

    // Max number of boxes in the boxes array
    protected int numMaxBoxes;

    // Indices that are not used by any boxes
    protected final ArrayList<Integer> freeBoxIndices;

    // Array that contains all the AABB boxes of the broad-phase
    protected BoxAABB[] boxes;

    // Array of end-points on the three axis
    protected final EndPoint[][] endPoints = {null, null, null};

    // Map a body pointer to a box index
    protected final Map<CollisionBody, Integer> mapBodyToBoxIndex;

    // Constructor
    public SweepAndPruneAlgorithm() {
        numBoxes = 0;
        numMaxBoxes = 0;
        boxes = null;
        freeBoxIndices = new ArrayList<>();
        mapBodyToBoxIndex = new HashMap<>();
    }

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
