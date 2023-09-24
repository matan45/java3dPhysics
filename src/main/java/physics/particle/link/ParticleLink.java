package physics.particle.link;

import math.Vector3f;
import physics.particle.Particle;
import physics.particle.contact.ParticleContactGenerator;

public abstract class ParticleLink implements ParticleContactGenerator {

    private Particle[] particle;

    public ParticleLink() {
        this.particle = new Particle[2];
    }

    protected float currentLength() {
        Vector3f relativePos = particle[0].getPosition().sub(particle[1].getPosition());
        return relativePos.length();
    }

    public Particle[] getParticle() {
        return particle;
    }

    public void setParticle(Particle[] particle) {
        this.particle = particle;
    }
}
