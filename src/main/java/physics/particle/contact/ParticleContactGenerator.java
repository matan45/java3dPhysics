package physics.particle.contact;

public interface ParticleContactGenerator {
    boolean addContact(ParticleContact contact, int limit);
}
