package physics.rigidBody;

import collisionDetection.narrowPhase.collisionResult.CollisionResult;

public interface SolveCollisionDispatcher {
    void solve(RigidBody bodyA, RigidBody bodyB, CollisionResult collisionResult);
}
