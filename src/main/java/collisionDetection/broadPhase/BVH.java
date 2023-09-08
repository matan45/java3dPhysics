package collisionDetection.broadPhase;

import collisionDetection.primitive.Ray;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class BVH implements BroadPhase {

    private BVHNode root;

    public BVH() {
        root = null;
    }

    @Override
    public void insert(BPBox obj) {
        if (root == null) {
            root = new BVHNode(obj);
        } else {
            root.insert(obj);
        }
    }

    @Override
    public void remove(BPBox obj) {
        if (root != null) {
            root.remove(obj);
        }
    }

    @Override
    public void update(BPBox obj) {
        remove(obj);
        insert(obj);
    }

    @Override
    public Set<BPPairs> query() {
        Set<BPPairs> collisionPairs = new HashSet<>();
        if (root != null) {
            root.query(collisionPairs);
        }
        return collisionPairs;
    }

    @Override
    public Set<BPBox> query(Ray ray) {
        Set<BPBox> potentialCollisions = new HashSet<>();
        root.query(ray, potentialCollisions);
        return potentialCollisions;
    }

    @Override
    public Set<BPBox> query(BPBox obj) {
        Set<BPBox> potentialCollisions = new HashSet<>();
        root.query(obj, potentialCollisions);
        return potentialCollisions;
    }

    @Override
    public void clear() {
        root = null; // Set the root node to null to clear the BVH.
    }

    @Override
    public void addAll(List<BPBox> boxes) {
        for (BPBox bpBox : boxes)
            insert(bpBox);
    }

    @Override
    public void updateAll(List<BPBox> boxes) {
        for (BPBox bpBox : boxes)
            update(bpBox);
    }

    @Override
    public void removeAll(List<BPBox> boxes) {
        for (BPBox bpBox : boxes)
            remove(bpBox);
    }
}
