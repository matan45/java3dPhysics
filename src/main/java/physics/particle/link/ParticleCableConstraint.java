package physics.particle.link;

import math.Vector3f;
import physics.particle.Particle;
import physics.particle.contact.ParticleContact;

import java.util.ArrayList;
import java.util.List;

public class ParticleCableConstraint extends ParticleConstraint {

    private float maxLength;


    private float restitution;

    public ParticleCableConstraint(float maxLength, float restitution) {
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
    public List<ParticleContact> getContacts(int limit) {
        // Find the length of the cable
        float length = currentLength();

        // Check if we're over-extended
        if (length < maxLength) {
            return new ArrayList<>();
        }

        ParticleContact contact = new ParticleContact();

        contact.setParticle(new Particle[]{getParticle()});

        // Calculate the normal
        Vector3f normal = getAnchor().sub(getParticle().getPosition());
        contact.setContactNormal(normal.normalize());
        contact.setPenetration(length - maxLength);
        contact.setRestitution(getRestitution());

        return List.of(contact);
    }
}
