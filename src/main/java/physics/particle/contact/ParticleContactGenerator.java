package physics.particle.contact;

import java.util.List;

public interface ParticleContactGenerator {
    List<ParticleContact> getContacts(int limit);
}
