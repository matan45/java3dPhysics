package physics.particle.link;

import math.Vector3f;
import physics.particle.Particle;
import physics.particle.contact.ParticleContactGenerator;

public abstract class ParticleConstraint implements ParticleContactGenerator {
    private Particle particle;

    private Vector3f anchor;

    public ParticleConstraint() {
        this.particle = new Particle();
        this.anchor = new Vector3f();
    }

    protected float currentLength() {
        Vector3f relativePos = particle.getPosition().sub(anchor);
        return relativePos.length();
    }

    public Particle getParticle() {
        return particle;
    }

    public void setParticle(Particle particle) {
        this.particle = particle;
    }

    public Vector3f getAnchor() {
        return anchor;
    }

    public void setAnchor(Vector3f anchor) {
        this.anchor = anchor;
    }
}
