package physics.particle;

public interface ParticleForceGenerator {
    void updateForce(Particle particle, float duration);
}
