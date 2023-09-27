package physics.particle.link;

import math.Vector3f;
import physics.particle.contact.ParticleContact;

import java.util.ArrayList;
import java.util.List;

public class ParticleRod extends ParticleLink {

    private float length;

    public ParticleRod(float length) {
        super();
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

        contact.setParticle(getParticle());

        // Calculate the normal
        Vector3f normal = getParticle()[1].getPosition().sub(getParticle()[0].getPosition());

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
