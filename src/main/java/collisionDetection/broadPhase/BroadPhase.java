package collisionDetection.broadPhase;

import collisionDetection.primitive.Ray;

import java.util.List;
import java.util.Set;

public interface BroadPhase {
    void insert(BPBox obj);

    void remove(BPBox obj);

    void update(BPBox obj);

    Set<BPPairs> query();

    Set<BPBox> query(Ray ray);

    Set<BPBox> query(BPBox obj);

    void clear();

    void addAll(List<BPBox> boxes);

    void updateAll(List<BPBox> boxes);
    void removeAll(List<BPBox> boxes);
}
