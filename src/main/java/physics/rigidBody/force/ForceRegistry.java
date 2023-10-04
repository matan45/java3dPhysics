package physics.rigidBody.force;

import java.util.ArrayList;
import java.util.List;

public class ForceRegistry {
    private final List<ForceRegistration> forceRegistrations;

    public ForceRegistry() {
        forceRegistrations = new ArrayList<>();
    }

    public void add(ForceRegistration forceRegistration) {
        forceRegistrations.add(forceRegistration);
    }

    public void remove(ForceRegistration forceRegistration) {
        forceRegistrations.remove(forceRegistration);
    }


    public void clear() {
        forceRegistrations.clear();
    }


    public void updateForces(float duration) {
        for (ForceRegistration forceRegistration : forceRegistrations) {
            forceRegistration.forceGenerator().updateForce(forceRegistration.body(), duration);
        }
    }
}
