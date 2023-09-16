package collisionDetection;

import collisionDetection.broadPhase.BPBox;
import collisionDetection.broadPhase.BroadPhase;
import collisionDetection.narrowPhase.cd.CDSatGjk;
import collisionDetection.narrowPhase.gjk.GJK;
import collisionDetection.narrowPhase.rc.RayCast;
import collisionDetection.narrowPhase.sat.SAT;

public class CDEngine {
    private final BroadPhase broadPhase;
    private final SAT sat;
    private final GJK gjk;
    private final CDSatGjk cdSatGjk;
    private final RayCast rayCast;
    private static CDEngine cdEngine;

    private CDEngine(BroadPhase broadPhase) {
        this.broadPhase = broadPhase;
        sat = new SAT();
        gjk = new GJK();
        cdSatGjk = new CDSatGjk();
        rayCast=new RayCast();
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

    public void remove(BPBox bpBox) {
        broadPhase.remove(bpBox);
    }

    public void update(BPBox bpBox) {
        broadPhase.update(bpBox);
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

    public CDSatGjk getCdSatGjk() {
        return cdSatGjk;
    }

    public RayCast getRayCast() {
        return rayCast;
    }
}
