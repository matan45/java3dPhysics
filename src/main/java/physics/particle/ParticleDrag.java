package physics.particle;

import math.Vector3f;

public class ParticleDrag implements ParticleForceGenerator {

    private final float k1;

    private final float k2;

    public ParticleDrag(float k1, float k2) {
        this.k1 = k1;
        this.k2 = k2;
    }

    @Override
    public void updateForce(Particle particle, float duration) {
        Vector3f force = particle.getForceAccumulate();

        // Calculate the total drag coefficient
        float dragCoeff = force.length();
        dragCoeff = k1 * dragCoeff + k2 * dragCoeff * dragCoeff;

        // Calculate the final force and apply it
        force = force.normalize().mul(-dragCoeff);
        particle.addForce(force);
    }
}
