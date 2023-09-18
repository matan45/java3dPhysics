package collisionDetection;

import collisionDetection.broadPhase.BPBox;
import collisionDetection.broadPhase.BPPairs;
import collisionDetection.broadPhase.BroadPhase;
import collisionDetection.narrowPhase.Shape;
import collisionDetection.narrowPhase.cd.CDSatGjk;
import collisionDetection.narrowPhase.collisionResult.CollisionResult;
import collisionDetection.narrowPhase.gjk.GJK;
import collisionDetection.narrowPhase.gjk.GJKSupport;
import collisionDetection.narrowPhase.rc.RayCast;
import collisionDetection.narrowPhase.sat.SAT;
import collisionDetection.narrowPhase.sat.SATSupport;
import collisionDetection.primitive.*;

import java.util.List;
import java.util.Set;

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
        if (shape1 instanceof SATSupport && shape2 instanceof SATSupport)
            return sat.isCollide((SATSupport) shape1, (SATSupport) shape2);
        else if (shape1 instanceof GJKSupport && shape2 instanceof GJKSupport)
            return gjk.isCollide((GJKSupport) shape1, (GJKSupport) shape2);
        else if (shape1 instanceof SATSupport)
            return getCollisionResult(shape2, (SATSupport) shape1);
        else if (shape2 instanceof SATSupport)
            return getCollisionResult(shape1, (SATSupport) shape2);
        else if (shape1 instanceof Plane)
            return getCollisionResult(shape2, (Plane) shape1);
        else if (shape2 instanceof Plane)
            return getCollisionResult(shape1, (Plane) shape2);

        throw new IllegalStateException("Unexpected values: " + shape2 + " " + shape1);
    }

    private CollisionResult getCollisionResult(Shape shape2, Plane shape1) {
        return switch (shape2) {
            case Sphere s -> cdSatGjk.isCollide(shape1, s);
            case Cylinder c -> cdSatGjk.isCollide(shape1, c);
            case Capsule c -> cdSatGjk.isCollide(shape1, c);
            case AABB a -> cdSatGjk.isCollide(shape1, a);
            case OBB b -> cdSatGjk.isCollide(shape1, b);
            case Line l -> cdSatGjk.isCollide(shape1, l);
            case Triangle t -> cdSatGjk.isCollide(shape1, t);
            case ConvexPolyhedron c -> cdSatGjk.isCollide(shape1, c);
            default -> throw new IllegalStateException("Unexpected value: " + shape2);
        };
    }

    private CollisionResult getCollisionResult(Shape shape2, SATSupport shape1) {
        return switch (shape2) {
            case Plane p -> cdSatGjk.isCollide(p, shape1);
            case Sphere s -> cdSatGjk.isCollide(s, shape1);
            case Cylinder c -> cdSatGjk.isCollide(c, shape1);
            case Capsule c -> cdSatGjk.isCollide(c, shape1);
            default -> throw new IllegalStateException("Unexpected value: " + shape2);
        };
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
