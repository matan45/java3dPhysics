package collision.broadphase;

import body.CollisionBody;
import collision.shapes.AABB;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class SweepAndPruneAlgorithm extends BroadPhaseAlgorithm {
    private static final int INVALID_INDEX = Integer.MAX_VALUE;
    private static final int NB_SENTINELS = 2;
    private BoxAABB[] mBoxes = null;
    private final EndPoint[][] mEndPoints = {null, null, null};
    private int mNbBoxes = 0;
    private int mNbMaxBoxes = 0;
    protected final List<Integer> mFreeBoxIndices = new ArrayList<>();
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
