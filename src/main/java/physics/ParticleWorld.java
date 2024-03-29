package physics;

import physics.particle.Particle;
import physics.particle.ParticleForceRegistry;
import physics.particle.contact.ParticleContact;
import physics.particle.contact.ParticleContactGenerator;
import physics.particle.contact.ParticleContactResolver;

import java.util.ArrayList;
import java.util.List;

public class ParticleWorld {
    private final List<Particle> particles;
    private final List<ParticleContactGenerator> particleContacts;
    private final boolean calculateIterations;
    private final ParticleForceRegistry particleForceRegistry;
    private final ParticleContactResolver particleContactResolver;
    private final int maxContacts;

    public ParticleWorld(int iterations, int maxContacts) {
        this.maxContacts = maxContacts;
        this.particles = new ArrayList<>();
        this.particleContacts = new ArrayList<>();
        this.calculateIterations = (iterations == 0);
        this.particleForceRegistry = new ParticleForceRegistry();
        this.particleContactResolver = new ParticleContactResolver(iterations);
    }

    private void integrate(float duration) {
        particles.forEach(p -> p.integrate(duration));
    }


    public void runPhysics(float duration) {
        particleForceRegistry.updateForces(duration);
        integrate(duration);
        ParticleContact[] contactArray = particleContacts.stream()
                .map(p -> p.getContacts(maxContacts))
                .flatMap(List::stream)
                .toList()
                .toArray(new ParticleContact[0]);
        int usedContacts = maxContacts - contactArray.length;
        if (usedContacts > 0) {
            if (calculateIterations) particleContactResolver.setIterations(usedContacts * 2);
            particleContactResolver.resolveContacts(contactArray, duration);
        }
    }


    public void startFrame() {
        particles.forEach(Particle::clearAccumulator);
    }

    public List<Particle> getParticles() {
        return particles;
    }

    public List<ParticleContactGenerator> getContactGenerators() {
        return particleContacts;
    }


    public ParticleForceRegistry getForceRegistry() {
        return particleForceRegistry;
    }
}
