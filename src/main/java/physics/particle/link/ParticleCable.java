package physics.particle.link;

import math.Vector3f;
import physics.particle.contact.ParticleContact;

public class ParticleCable extends ParticleLink {

    /**
     * Holds the maximum length of the cable.
     */
    private float maxLength;

    /**
     * Holds the restitution (bounciness) of the cable.
     */
    private float restitution;

    public ParticleCable(float maxLength, float restitution) {
        super();
        this.maxLength = maxLength;
        this.restitution = restitution;
    }

    public float getMaxLength() {
        return maxLength;
    }

    public void setMaxLength(float maxLength) {
        this.maxLength = maxLength;
    }

    public float getRestitution() {
        return restitution;
    }

    public void setRestitution(float restitution) {
        this.restitution = restitution;
    }

    @Override
    public boolean addContact(ParticleContact contact, int limit) {
        // Find the length of the cable
        float length = currentLength();

        // Check if we're over-extended
        if (length < maxLength) {
            return false;
        }

        contact.setParticle(getParticle());

        // Calculate the normal
        Vector3f normal = getParticle()[1].getPosition().sub(getParticle()[0].getPosition());
        contact.setContactNormal(normal.normalize());

        contact.setPenetration(length - maxLength);
        contact.setRestitution(restitution);

        return true;
    }
}
