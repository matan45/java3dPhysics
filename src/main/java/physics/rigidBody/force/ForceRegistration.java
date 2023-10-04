package physics.rigidBody.force;

import physics.rigidBody.RigidBody;

public record ForceRegistration (RigidBody body, ForceGenerator forceGenerator){
}
