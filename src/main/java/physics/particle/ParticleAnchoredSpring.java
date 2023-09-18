package physics.particle;

import math.Vector3f;

public class ParticleAnchoredSpring implements ParticleForceGenerator {

    private final Vector3f anchor;

    private float springConstant;

    private float restLength;

    public ParticleAnchoredSpring() {
        this.anchor = new Vector3f();
    }

    public ParticleAnchoredSpring(Vector3f anchor, float springConstant, float restLength) {
        this.anchor = anchor;
        this.springConstant = springConstant;
        this.restLength = restLength;
    }

    public Vector3f getAnchor() {
        return anchor;
    }

    public float getSpringConstant() {
        return springConstant;
    }

    public float getRestLength() {
        return restLength;
    }

    @Override
    public void updateForce(Particle particle, float duration) {
        // Calculate the vector of the spring
        Vector3f force = particle.getPosition();
        force = force.sub(anchor);

        // Calculate the magnitude of the force
        float magnitude = force.length();
        magnitude = (restLength - magnitude) * springConstant;

        // Calculate the final force and apply it
        force = force.normalize().mul(magnitude);
        particle.addForce(force);
    }
}
