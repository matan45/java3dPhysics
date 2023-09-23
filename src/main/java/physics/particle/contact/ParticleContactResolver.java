package physics.particle.contact;

import math.Vector3f;

public class ParticleContactResolver {
    /**
     * Holds the number of iterations allowed.
     */
    private int iterations;

    /**
     * This is a performance tracking value - we keep a record
     * of the actual number of iterations used.
     */
    private int iterationsUsed;

    public ParticleContactResolver(int iterations) {
        this.iterations = iterations;
    }

    public void setIterations(int iterations) {
        this.iterations = iterations;
    }

    public int getIterationsUsed() {
        return iterationsUsed;
    }

    public void resolveContacts(ParticleContact[] contactArray, float duration) {
        int i;
        int numContacts = contactArray.length;
        iterationsUsed = 0;
        while (iterationsUsed < iterations) {
            // Find the contact with the largest closing velocity;
            float max = Float.MAX_VALUE;
            int maxIndex = numContacts;
            for (i = 0; i < numContacts; i++) {
                float sepVel = contactArray[i].calculateSeparatingVelocity();
                if (sepVel < max && (sepVel < 0 || contactArray[i].getPenetration() > 0)) {
                    max = sepVel;
                    maxIndex = i;
                }
            }

            // Do we have anything worth resolving?
            if (maxIndex == numContacts) break;

            // Resolve this contact
            contactArray[maxIndex].resolve(duration);

            // Update the interpenetrations for all particles
            Vector3f[] move = contactArray[maxIndex].getParticleMovement();
            for (i = 0; i < numContacts; i++) {
                if (contactArray[i].getParticle()[0].equals(contactArray[maxIndex].getParticle()[0])) {
                    float penetration = contactArray[i].getPenetration();
                    float contact = contactArray[i].getContactNormal().dot(move[0]);
                    contactArray[i].setPenetration(penetration - contact);
                } else if (contactArray[i].getParticle()[0].equals(contactArray[maxIndex].getParticle()[1])) {
                    float penetration = contactArray[i].getPenetration();
                    float contact = contactArray[i].getContactNormal().dot(move[1]);
                    contactArray[i].setPenetration(penetration - contact);
                }
                if (contactArray[i].getParticle()[1] != null) {
                    if (contactArray[i].getParticle()[1].equals(contactArray[maxIndex].getParticle()[0])) {
                        float penetration = contactArray[i].getPenetration();
                        float contact = contactArray[i].getContactNormal().dot(move[0]);
                        contactArray[i].setPenetration(penetration + contact);
                    } else if (contactArray[i].getParticle()[1].equals(contactArray[maxIndex].getParticle()[1])) {
                        float penetration = contactArray[i].getPenetration();
                        float contact = contactArray[i].getContactNormal().dot(move[1]);
                        contactArray[i].setPenetration(penetration + contact);
                    }
                }
            }

            iterationsUsed++;
        }
    }
}
