package collisionDetection;

import collisionDetection.broadPhase.BPBox;
import collisionDetection.broadPhase.BPPairs;
import collisionDetection.broadPhase.BroadPhase;
import collisionDetection.narrowPhase.Shape;
import collisionDetection.narrowPhase.collisionResult.CollisionResult;
import collisionDetection.narrowPhase.gjk.GJK;
import collisionDetection.narrowPhase.gjk.GJKSupport;
import collisionDetection.narrowPhase.rc.RayCast;
import collisionDetection.narrowPhase.sat.SAT;
import collisionDetection.narrowPhase.sat.SATSupport;
import collisionDetection.primitive.Ray;

import java.util.List;
import java.util.Set;

public class CDEngine {
    private final BroadPhase broadPhase;
    private final SAT sat;
    private final GJK gjk;
    private final RayCast rayCast;
    private static CDEngine cdEngine;

    private CDEngine(BroadPhase broadPhase) {
        this.broadPhase = broadPhase;
        sat = new SAT();
        gjk = new GJK();
        rayCast = new RayCast();
    }

    public static void init(BroadPhase broadPhase) {
        cdEngine = new CDEngine(broadPhase);
    }

    public static CDEngine getCdEngine() {
        return cdEngine;
    }

    public void add(BPBox bpBox) {
        broadPhase.insert(bpBox);
    }

    public void addAll(List<BPBox> boxes) {
        broadPhase.addAll(boxes);
    }

    public void remove(BPBox bpBox) {
        broadPhase.remove(bpBox);
    }

    public void removeAll(List<BPBox> boxes) {
        broadPhase.removeAll(boxes);
    }

    public void update(List<BPBox> boxes) {
        broadPhase.updateAll(boxes);
    }

    public void update(BPBox bpBox) {
        broadPhase.update(bpBox);
    }

    public void updateAll(List<BPBox> boxes) {
        broadPhase.updateAll(boxes);
    }

    public Set<BPBox> query(BPBox bpBox) {
        return broadPhase.query(bpBox);
    }

    public Set<BPBox> query(Ray ray) {
        return broadPhase.query(ray);
    }

    public CollisionResult solve(BPPairs bpPairs) {
        Shape shape1 = bpPairs.getBpBox1().getShape();
        Shape shape2 = bpPairs.getBpBox2().getShape();
        if (shape1 instanceof GJKSupport && shape2 instanceof GJKSupport)
            return gjk.isCollide((GJKSupport) shape1, (GJKSupport) shape2);
        else if (shape1 instanceof SATSupport && shape2 instanceof SATSupport)
            return sat.isCollide((SATSupport) shape1, (SATSupport) shape2);

        throw new IllegalStateException("Unexpected values: " + shape2 + " : " + shape1);
    }

    public void clear() {
        broadPhase.clear();
    }

    public SAT getSat() {
        return sat;
    }

    public GJK getGjk() {
        return gjk;
    }

    public RayCast getRayCast() {
        return rayCast;
    }
}
