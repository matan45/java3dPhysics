package physics.particle.contact;

import math.Vector3f;
import physics.particle.Particle;

public class ParticleContact {
    /**
     * Holds the particles that are involved in the contact. The
     * second of these can be NULL, for contacts with the scenery.
     */
    private Particle[] particle;

    /**
     * Holds the normal restitution coefficient at the contact.
     */
    private float restitution;

    /**
     * Holds the direction of the contact in world coordinates.
     */
    private Vector3f contactNormal;

    /**
     * Holds the depth of penetration at the contact.
     */
    private float penetration;

    /**
     * Holds the amount each particle is moved by during interpenetration
     * resolution.
     */
    private Vector3f[] particleMovement;

    public ParticleContact() {
        particle = new Particle[2];
        particleMovement = new Vector3f[2];
    }



    public Particle[] getParticle() {
        return particle;
    }

    public void setParticle(Particle[] particle) {
        this.particle = particle;
    }

    public float getRestitution() {
        return restitution;
    }

    public void setRestitution(float restitution) {
        this.restitution = restitution;
    }

    public Vector3f getContactNormal() {
        return contactNormal;
    }

    public void setContactNormal(Vector3f contactNormal) {
        this.contactNormal = contactNormal;
    }

    public float getPenetration() {
        return penetration;
    }

    public void setPenetration(float penetration) {
        this.penetration = penetration;
    }

    public Vector3f[] getParticleMovement() {
        return particleMovement;
    }

    public void setParticleMovement(Vector3f[] particleMovement) {
        this.particleMovement = particleMovement;
    }

    public void resolve(float duration) {
        resolveVelocity(duration);
        resolveInterpenetration();
    }

    public float calculateSeparatingVelocity() {
        Vector3f relativeVelocity = particle[0].getVelocity();
        if (particle[1] != null)
            relativeVelocity = relativeVelocity.sub(particle[1].getVelocity());
        return relativeVelocity.dot(contactNormal);
    }

    private void resolveVelocity(float duration) {
        // Find the velocity in the direction of the contact
        float separatingVelocity = calculateSeparatingVelocity();

        // Check if it needs to be resolved
        if (separatingVelocity > 0) {
            // The contact is either separating, or stationary - there's
            // no impulse required.
            return;
        }

        // Calculate the new separating velocity
        float newSepVelocity = -separatingVelocity * restitution;

        // Check the velocity build-up due to acceleration only
        Vector3f accCausedVelocity = particle[0].getAcceleration();
        if (particle[1] != null)
            accCausedVelocity = accCausedVelocity.sub(particle[1].getAcceleration());
        float accCausedSepVelocity = accCausedVelocity.dot(contactNormal.mul(duration));

        // If we've got a closing velocity due to acceleration build-up,
        // remove it from the new separating velocity
        if (accCausedSepVelocity < 0) {
            newSepVelocity += restitution * accCausedSepVelocity;

            // Make sure we haven't removed more than was
            // there to remove.
            if (newSepVelocity < 0) newSepVelocity = 0;
        }

        float deltaVelocity = newSepVelocity - separatingVelocity;

        // We apply the change in velocity to each object in proportion to
        // their inverse mass (i.e. those with lower inverse mass [higher
        // actual mass] get less change in velocity)..
        float totalInverseMass = particle[0].getInverseMass();
        if (particle[1] != null) totalInverseMass += particle[1].getInverseMass();

        // If all particles have infinite mass, then impulses have no effect
        if (totalInverseMass <= 0) return;

        // Calculate the impulse to apply
        float impulse = deltaVelocity / totalInverseMass;

        // Find the amount of impulse per unit of inverse mass
        Vector3f impulsePerIMass = contactNormal.mul(impulse);

        // Apply impulses: they are applied in the direction of the contact,
        // and are proportional to the inverse mass.
        particle[0].setVelocity(particle[0].getVelocity()
                .add(impulsePerIMass.mul(particle[0].getInverseMass())));

        if (particle[1] != null) {
            // Particle 1 goes in the opposite direction
            particle[1].setVelocity(particle[1].getVelocity()
                    .add(impulsePerIMass.mul(-particle[1].getInverseMass())));
        }

    }

    private void resolveInterpenetration() {
        // If we don't have any penetration, skip this step.
        if (penetration <= 0) return;

        // The movement of each object is based on their inverse mass, so
        // total that.
        float totalInverseMass = particle[0].getInverseMass();
        if (particle[1] != null) totalInverseMass += particle[1].getInverseMass();

        // If all particles have infinite mass, then we do nothing
        if (totalInverseMass <= 0) return;

        // Find the amount of penetration resolution per unit of inverse mass
        Vector3f movePerIMass = contactNormal.mul((penetration / totalInverseMass));

        // Calculate the the movement amounts
        particleMovement[0] = movePerIMass.mul(particle[0].getInverseMass());
        if (particle[1] != null) {
            particleMovement[1] = movePerIMass.mul(-particle[1].getInverseMass());
        } else {
            particleMovement[1].clear();
        }

        // Apply the penetration resolution
        particle[0].setPosition(particle[0].getPosition().add(particleMovement[0]));
        if (particle[1] != null) {
            particle[1].setPosition(particle[1].getPosition().add(particleMovement[1]));
        }
    }
}
