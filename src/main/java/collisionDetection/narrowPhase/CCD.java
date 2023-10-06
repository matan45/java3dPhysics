package collisionDetection.narrowPhase;

import collisionDetection.narrowPhase.collisionResult.CollisionResult;
import collisionDetection.narrowPhase.gjk.GJK;
import collisionDetection.narrowPhase.gjk.GJKSupport;
import collisionDetection.narrowPhase.sat.SAT;
import collisionDetection.narrowPhase.sat.SATSupport;
import math.Vector3f;
import physics.rigidBody.RigidBody;

public class CCD {
    //TODO add shapes model matrix

    private final GJK gjk;
    private final SAT sat;

    public CCD(GJK gjk, SAT sat) {
        this.gjk = gjk;
        this.sat = sat;
    }

    public CollisionResult gjk(RigidBody rigidBodyA, RigidBody rigidBodyB, float dt, int numSubSteps) {
        for (int subStep = 0; subStep <= numSubSteps; subStep++) {
            float currentTime = subStep * (dt / numSubSteps);

            // Interpolate positions at the current sub-step
            interpolatePositions(rigidBodyA, rigidBodyB, currentTime);

            CollisionResult result = gjk.isCollide((GJKSupport) rigidBodyA.getColliderShape(), (GJKSupport) rigidBodyA.getColliderShape());
            if (result.isColliding())
                return result;

        }
        return new CollisionResult();
    }

    public CollisionResult sat(RigidBody rigidBodyA, RigidBody rigidBodyB, float dt, int numSubSteps) {
        for (int subStep = 0; subStep <= numSubSteps; subStep++) {
            float currentTime = subStep * (dt / numSubSteps);

            // Interpolate positions at the current sub-step
            interpolatePositions(rigidBodyA, rigidBodyB, currentTime);
            CollisionResult result = sat.isCollide((SATSupport) rigidBodyA.getColliderShape(), (SATSupport) rigidBodyA.getColliderShape());
            if (result.isColliding())
                return result;
        }
        return new CollisionResult();
    }

    private static void interpolatePositions(RigidBody rigidBodyA, RigidBody rigidBodyB, float dt) {
        // Interpolate positions of rigid bodies at the given time
        // Use initial positions, velocities, and currentTime to compute positions

        Vector3f initialPositionA = rigidBodyA.getPosition();
        Vector3f initialVelocityA = rigidBodyA.getVelocity();

        Vector3f initialPositionB = rigidBodyB.getPosition();
        Vector3f initialVelocityB = rigidBodyB.getVelocity();

        // Calculate the new positions
        Vector3f newPositionA = new Vector3f(
                initialPositionA.x + initialVelocityA.x * dt,
                initialPositionA.y + initialVelocityA.y * dt,
                initialPositionA.z + initialVelocityA.z * dt
        );

        Vector3f newPositionB = new Vector3f(
                initialPositionB.x + initialVelocityB.x * dt,
                initialPositionB.y + initialVelocityB.y * dt,
                initialPositionB.z + initialVelocityB.z * dt
        );

        // Update the positions of the rigid bodies
        rigidBodyA.setPosition(newPositionA);
        rigidBodyB.setPosition(newPositionB);
    }
}
