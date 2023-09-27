package physics.particle.force;

import math.Vector3f;
import physics.particle.Particle;
import physics.particle.ParticleForceGenerator;

public class ParticleFakeSpring implements ParticleForceGenerator {

    private final Vector3f anchor;

    private final float springConstant;

    private final float damping;

    public ParticleFakeSpring(Vector3f anchor, float springConstant, float damping) {
        this.anchor = anchor;
        this.springConstant = springConstant;
        this.damping = damping;
    }

    @Override
    public void updateForce(Particle particle, float duration) {
        // Check that we do not have infinite mass
        if (!particle.isFiniteMass()) return;

        // Calculate the relative position of the particle to the anchor
        Vector3f position = particle.getPosition();
        position = position.sub(anchor);

        // Calculate the constants and check they are in bounds.
        float gamma = (float) (0.5f * Math.sqrt(4 * springConstant - damping * damping));
        if (gamma == 0.0f) return;
        Vector3f c = position.mul(damping / (2.0f * gamma)).add(particle.getVelocity().mul(1.0f / gamma));

        // Calculate the target position
        Vector3f target = position.mul((float) Math.cos(gamma * duration)).add(c.mul((float) Math.sin(gamma * duration)));
        target = target.mul((float) Math.exp(-0.5f * duration * damping));

        // Calculate the resulting acceleration and therefore the force
        Vector3f accel = target.sub(position).mul(1.0f / (duration * duration))
                .sub(particle.getVelocity().mul(1.0f / duration));
        particle.addForce(accel.mul(particle.getMass()));
    }
}
