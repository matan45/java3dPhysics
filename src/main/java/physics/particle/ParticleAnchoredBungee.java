package physics.particle;

import math.Vector3f;

public class ParticleAnchoredBungee extends ParticleAnchoredSpring {

    public ParticleAnchoredBungee() {
        super();
    }

    public ParticleAnchoredBungee(Vector3f anchor, float springConstant, float restLength) {
        super(anchor, springConstant, restLength);
    }

    @Override
    public void updateForce(Particle particle, float duration) {
        // Calculate the vector of the spring
        Vector3f force = particle.getPosition();
        force = force.sub(getAnchor());

        // Calculate the magnitude of the force
        float magnitude = force.length();
        if (magnitude < getRestLength()) return;

        magnitude = magnitude - getRestLength();
        magnitude *= getSpringConstant();

        // Calculate the final force and apply it
        force = force.normalize().mul(-magnitude);
        particle.addForce(force);
    }
}
