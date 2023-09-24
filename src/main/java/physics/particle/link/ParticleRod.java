package physics.particle.link;

import math.Vector3f;
import physics.particle.contact.ParticleContact;

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
    public boolean addContact(ParticleContact contact, int limit) {
        // Find the length of the rod
        float currentLen = currentLength();

        // Check if we're over-extended
        if (currentLen == length) {
            return false;
        }

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

        return true;
    }
}
