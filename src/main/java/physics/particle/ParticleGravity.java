package physics.particle;

import math.Vector3f;

public class ParticleGravity implements ParticleForceGenerator {

    private final Vector3f gravity;

    public ParticleGravity(Vector3f gravity) {
        this.gravity = gravity;
    }

    @Override
    public void updateForce(Particle particle, float duration) {
        // Check that we do not have infinite mass
        if (!particle.hasFiniteMass()) return;

        // Apply the mass-scaled force to the particle
        particle.addForce(gravity.mul(particle.getMass()));

    }
}
