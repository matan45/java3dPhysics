package physics.rigidBody.force;

import physics.rigidBody.RigidBody;

public interface ForceGenerator {
    void updateForce(RigidBody body, float duration);
}
