package collisionDetection.broadPhase;

import collisionDetection.primitive.Ray;

import java.util.*;
import java.util.stream.Collectors;

public class SAP implements BroadPhase {
    private final List<BPBox> xAxis;
    private final List<BPBox> yAxis;
    private final List<BPBox> zAxis;


    public SAP() {
        xAxis = new ArrayList<>();
        yAxis = new ArrayList<>();
        zAxis = new ArrayList<>();
    }

    @Override
    public void insert(BPBox obj) {
        xAxis.add(obj);
        yAxis.add(obj);
        zAxis.add(obj);

        // Sort the lists based on the object's positions along each axis
        xAxis.sort(Comparator.comparing(o -> o.getCenter().x));
        yAxis.sort(Comparator.comparing(o -> o.getCenter().y));
        zAxis.sort(Comparator.comparing(o -> o.getCenter().z));
    }

    @Override
    public void remove(BPBox obj) {
        xAxis.remove(obj);
        yAxis.remove(obj);
        zAxis.remove(obj);
    }

    @Override
    public void update(BPBox obj) {
        remove(obj);
        insert(obj);
    }

    @Override
    public Set<BPPairs> query() {
        Set<BPPairs> pairsSet = new HashSet<>();
        pairsSet.addAll(getFromList(xAxis));
        pairsSet.addAll(getFromList(yAxis));
        pairsSet.addAll(getFromList(zAxis));

        return pairsSet;
    }

    @Override
    public Set<BPBox> query(Ray ray) {
        return xAxis.stream()
                .filter(box -> BPBox.isCollide(ray, box))
                .collect(Collectors.toSet());
    }

    @Override
    public Set<BPBox> query(BPBox obj) {
        return xAxis.stream()
                .filter(b -> !b.getShape().equals(obj.getShape()))
                .filter(box -> BPBox.isCollide(box, obj))
                .collect(Collectors.toSet());
    }

    @Override
    public void clear() {
        xAxis.clear();
        yAxis.clear();
        zAxis.clear();
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

    private Set<BPPairs> getFromList(List<BPBox> bpBoxes) {
        Set<BPPairs> pairsSet = new HashSet<>();
        if (bpBoxes.size() > 1) {
            for (int i = 0; i < bpBoxes.size() - 1; i++) {
                if (BPBox.isCollide(bpBoxes.get(i), bpBoxes.get(i + 1)))
                    pairsSet.add(new BPPairs(bpBoxes.get(i), bpBoxes.get(i + 1)));
            }
        }
        return pairsSet;
    }
}
