package physics;

import collisionDetection.CDEngine;
import collisionDetection.broadPhase.BPPairs;
import collisionDetection.broadPhase.BroadPhase;
import collisionDetection.narrowPhase.collisionResult.CollisionResult;
import physics.rigidBody.RigidBody;
import physics.rigidBody.RigidBodySolver;
import physics.rigidBody.SolveCollisionDispatcher;

import java.util.Set;

public class PhysicsEngine {

    private final SolveCollisionDispatcher solveCollisionDispatcher;
    private final CDEngine cdEngine;

    public PhysicsEngine(BroadPhase broadPhase) {
        CDEngine.init(broadPhase);
        this.cdEngine = CDEngine.getCdEngine();
        this.solveCollisionDispatcher = new RigidBodySolver();
    }

    public void solve() {
        Set<BPPairs> bpPairs = cdEngine.query();
        for (BPPairs pairs : bpPairs) {
            RigidBody bodyA = pairs.getBpBox1().getBody();
            RigidBody bodyB = pairs.getBpBox2().getBody();
            CollisionResult result = cdEngine.solve(pairs);
            solveCollisionDispatcher.solve(bodyA, bodyB, result);
        }
    }
}
