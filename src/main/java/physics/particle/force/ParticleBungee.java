package physics.particle.force;

import math.Vector3f;
import physics.particle.Particle;
import physics.particle.ParticleForceGenerator;

public class ParticleBungee implements ParticleForceGenerator {

    private final Particle other;

    private final float springConstant;

    private final float restLength;

    public ParticleBungee(Particle other, float springConstant, float restLength) {
        this.other = other;
        this.springConstant = springConstant;
        this.restLength = restLength;
    }

    @Override
    public void updateForce(Particle particle, float duration) {
        // Calculate the vector of the spring
        Vector3f force = particle.getPosition();
        force = force.sub(other.getPosition());

        // Check if the bungee is compressed
        float magnitude = force.length();
        if (magnitude <= restLength) return;

        // Calculate the magnitude of the force
        magnitude = springConstant * (restLength - magnitude);

        // Calculate the final force and apply it
        force = force.normalize().mul(-magnitude);
        particle.addForce(force);
    }
}
