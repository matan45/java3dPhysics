package physics.particle.link;

import math.Vector3f;
import physics.particle.Particle;
import physics.particle.contact.ParticleContact;
import physics.particle.contact.ParticleContactGenerator;

import java.util.ArrayList;
import java.util.List;

public class GroundContacts implements ParticleContactGenerator {
    private List<Particle> particles;

    public GroundContacts(List<Particle> particles) {
        this.particles = particles;
    }

    public List<Particle> getParticles() {
        return particles;
    }

    public void setParticles(List<Particle> particles) {
        this.particles = particles;
    }

    @Override
    public List<ParticleContact> getContacts(int limit) {
        List<ParticleContact> contacts = new ArrayList<>();
        int count = 0;
        for (Particle p : particles) {
            float y = p.getPosition().getY();
            if (y < 0.0f && count < limit) {
                ParticleContact contact = new ParticleContact();
                contact.setContactNormal(Vector3f.YAxis);
                contact.setParticle(new Particle[]{p});
                contact.setPenetration(-y);
                contact.setRestitution(0.2f);
                contacts.add(contact);
                count++;
            }
        }
        return contacts;
    }
}
