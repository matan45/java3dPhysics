package collisionDetection.broadPhase;

import collisionDetection.primitive.Ray;

import java.util.*;
import java.util.function.Predicate;
import java.util.stream.Collectors;

import static math.Const.BP_MARGIN;

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
        xAxis.sort(Comparator.comparing(o -> o.getMin().x));
        yAxis.sort(Comparator.comparing(o -> o.getMin().y));
        zAxis.sort(Comparator.comparing(o -> o.getMin().z));
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
        Set<BPBox> bpBoxes = new HashSet<>();
        bpBoxes.addAll(getForRay(xAxis, ray));
        bpBoxes.addAll(getForRay(yAxis, ray));
        bpBoxes.addAll(getForRay(zAxis, ray));

        return bpBoxes;
    }

    @Override
    public Set<BPBox> query(BPBox obj) {
        Set<BPBox> bpBoxes = new HashSet<>();
        bpBoxes.addAll(getCloseBoxes(xAxis, obj));
        bpBoxes.addAll(getCloseBoxes(yAxis, obj));
        bpBoxes.addAll(getCloseBoxes(zAxis, obj));

        return bpBoxes;
    }

    private Set<BPBox> getCloseBoxes(List<BPBox> bpBoxes, BPBox obj) {
        Predicate<BPBox> min = box -> box.getMin().x - obj.getMin().x < BP_MARGIN;
        return bpBoxes.stream().filter(min).collect(Collectors.toSet());
    }

    @Override
    public void removeAll() {
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

    private Set<BPBox> getForRay(List<BPBox> boxes, Ray ray) {
        Set<BPBox> bpBoxes = new HashSet<>();
        for (BPBox bpBox : boxes) {
            if (BPBox.isCollide(ray, bpBox))
                bpBoxes.add(bpBox);
        }
        return bpBoxes;
    }
}
