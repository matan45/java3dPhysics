package physics;

import physics.rigidBody.force.ForceRegistry;

public class World {

    private final ForceRegistry forceRegistry;

    public World() {
        this.forceRegistry = new ForceRegistry();
    }
}
