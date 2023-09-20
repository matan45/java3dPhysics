package physics.particle;

import math.Vector3f;

public class ParticleBuoyancy implements ParticleForceGenerator {
    private final float maxDepth;

    private final float volume;

    private final float waterHeight;

    private final float liquidDensity;

    public ParticleBuoyancy(float maxDepth, float volume, float waterHeight, float liquidDensity) {
        this.maxDepth = maxDepth;
        this.volume = volume;
        this.waterHeight = waterHeight;
        this.liquidDensity = liquidDensity;
    }

    @Override
    public void updateForce(Particle particle, float duration) {
        // Calculate the submersion depth
        float depth = particle.getPosition().y;

        // Check if we're out of the water
        if (depth >= waterHeight + maxDepth) return;
        Vector3f force = new Vector3f();

        // Check if we're at maximum depth
        if (depth <= waterHeight - maxDepth) {
            force.y = liquidDensity * volume;
            particle.addForce(force);
            return;
        }

        // Otherwise we are partly submerged
        force.y = liquidDensity * volume *
                (depth - maxDepth - waterHeight) / (2 * maxDepth);
        particle.addForce(force);
    }
}
