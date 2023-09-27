package physics.particle.link;

import math.Vector3f;
import physics.particle.Particle;
import physics.particle.contact.ParticleContact;

import java.util.ArrayList;
import java.util.List;

public class ParticleRodConstraint extends ParticleConstraint {
    private float length;

    public ParticleRodConstraint(float length) {
        this.length = length;
    }

    public float getLength() {
        return length;
    }

    public void setLength(float length) {
        this.length = length;
    }

    @Override
    public List<ParticleContact> getContacts(int limit) {
        // Find the length of the rod
        float currentLen = currentLength();

        // Check if we're over-extended
        if (currentLen == length) {
            return new ArrayList<>();
        }
        ParticleContact contact = new ParticleContact();
        contact.setParticle(new Particle[]{getParticle()});

        // Calculate the normal
        Vector3f normal = getAnchor().sub(getParticle().getPosition());

        // The contact normal depends on whether we're extending or compressing
        if (currentLen > length) {
            contact.setContactNormal(normal.normalize());
            contact.setPenetration(currentLen - length);
        } else {
            contact.setContactNormal(normal.normalize().negate());
            contact.setPenetration(length - currentLen);
        }

        // Always use zero restitution (no bounciness)
        contact.setRestitution(0);

        return List.of(contact);
    }
}
