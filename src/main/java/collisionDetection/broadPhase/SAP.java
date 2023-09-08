package collisionDetection.broadPhase;

import collisionDetection.primitive.Ray;

import java.util.*;

public class SAP {
    private final List<BPBox> xAxis;
    private final List<BPBox> yAxis;
    private final List<BPBox> zAxis;


    public SAP() {
        xAxis = new ArrayList<>();
        yAxis = new ArrayList<>();
        zAxis = new ArrayList<>();
    }

    public void insert(BPBox obj) {
        xAxis.add(obj);
        yAxis.add(obj);
        zAxis.add(obj);

        // Sort the lists based on the object's positions along each axis
        xAxis.sort(Comparator.comparing(o -> o.getMin().x));
        yAxis.sort(Comparator.comparing(o -> o.getMin().y));
        zAxis.sort(Comparator.comparing(o -> o.getMin().z));
    }

    public void remove(BPBox obj) {
        xAxis.remove(obj);
        yAxis.remove(obj);
        zAxis.remove(obj);
    }

    public void update(BPBox obj) {
        remove(obj);
        insert(obj);
    }

    public Set<BPPairs> query() {
        Set<BPPairs> pairsSet = new HashSet<>();
        pairsSet.addAll(getFromList(xAxis));
        pairsSet.addAll(getFromList(yAxis));
        pairsSet.addAll(getFromList(zAxis));

        return pairsSet;
    }

    public Set<BPBox> query(Ray ray) {
        Set<BPBox> bpBoxes = new HashSet<>();
        bpBoxes.addAll(getForRay(xAxis, ray));
        bpBoxes.addAll(getForRay(yAxis, ray));
        bpBoxes.addAll(getForRay(zAxis, ray));

        return bpBoxes;
    }

    public void removeAll() {
        xAxis.clear();
        yAxis.clear();
        zAxis.clear();
    }

    public void addAll(List<BPBox> boxes) {
        for (BPBox bpBox : boxes)
            insert(bpBox);
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
