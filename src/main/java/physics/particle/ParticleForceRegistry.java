package physics.particle;

import java.util.ArrayList;
import java.util.List;

public class ParticleForceRegistry {

    private final List<ParticleForceRegistration> particleForceRegistrations;

    public ParticleForceRegistry() {
        particleForceRegistrations = new ArrayList<>();
    }

    public void add(ParticleForceRegistration particleForceRegistration) {
        particleForceRegistrations.add(particleForceRegistration);
    }

    public void remove(ParticleForceRegistration particleForceRegistration) {
        particleForceRegistrations.remove(particleForceRegistration);
    }


    public void clear() {
        particleForceRegistrations.clear();
    }


    public void updateForces(float duration) {
        for (ParticleForceRegistration particleForceRegistration : particleForceRegistrations) {
            particleForceRegistration.particleForceGenerator().updateForce(particleForceRegistration.particle(), duration);
        }
    }
}
