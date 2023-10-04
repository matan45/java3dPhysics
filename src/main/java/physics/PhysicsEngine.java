package physics;

import collisionDetection.CDEngine;
import collisionDetection.broadPhase.BPPairs;
import collisionDetection.broadPhase.BroadPhase;
import collisionDetection.narrowPhase.collisionResult.CollisionResult;
import physics.particle.ParticleForceRegistry;
import physics.rigidBody.force.ForceRegistry;
import physics.rigidBody.RigidBody;
import physics.rigidBody.RigidBodySolver;
import physics.rigidBody.SolveCollisionDispatcher;

import java.util.Set;

public class PhysicsEngine {

    private final SolveCollisionDispatcher solveCollisionDispatcher;
    private final CDEngine cdEngine;
    private final ParticleWorld particleWorld;
    private final World world;

    public PhysicsEngine(BroadPhase broadPhase, int iterations, int maxContacts) {
        CDEngine.init(broadPhase);
        this.cdEngine = CDEngine.getCdEngine();
        this.solveCollisionDispatcher = new RigidBodySolver();
        this.particleWorld = new ParticleWorld(iterations, maxContacts);
        this.world = new World();
    }

    public void solve() {
        Set<BPPairs> bpPairs = cdEngine.query();
        for (BPPairs pairs : bpPairs) {
            CollisionResult result = cdEngine.solve(pairs);
            if (result.isColliding()) {
                RigidBody bodyA = pairs.getBpBox1().getBody();
                RigidBody bodyB = pairs.getBpBox2().getBody();
                solveCollisionDispatcher.solve(bodyA, bodyB, result);
            }
        }
    }
}
