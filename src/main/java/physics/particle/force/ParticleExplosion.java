package physics.particle.force;

import math.Vector3f;
import physics.particle.Particle;
import physics.particle.ParticleForceGenerator;

public class ParticleExplosion implements ParticleForceGenerator {

    /**
     * Tracks how long the explosion has been in operation, used
     * for time-sensitive effects.
     */
    private float timePassed;

    /**
     * The location of the detonation of the weapon.
     */
    private Vector3f detonation;

    /**
     * The radius up to which objects implode in the first stage
     * of the explosion.
     */
    private float implosionMaxRadius;

    /**
     * The radius within which objects don't feel the implosion
     * force. Objects near to the detonation aren't sucked in by
     * the air implosion.
     */
    private float implosionMinRadius;

    /**
     * The length of time that objects spend imploding before the
     * concussion phase kicks in.
     */
    private float implosionDuration;

    /**
     * The maximal force that the implosion can apply. This should
     * be relatively small to avoid the implosion pulling objects
     * through the detonation point and out the other side before
     * the concussion wave kicks in.
     */
    private float implosionForce;

    /**
     * The speed that the shock wave is traveling, this is related
     * to the thickness below in the relationship:
     * <p>
     * thickness >= speed * minimum frame duration
     */
    private float shockwaveSpeed;

    /**
     * The shock wave applies its force over a range of distances,
     * this controls how thick. Faster waves require larger
     * thicknesses.
     */
    private float shockwaveThickness;

    /**
     * This is the force that is applied at the very centre of the
     * concussion wave on an object that is stationary. Objects
     * that are in front or behind of the wavefront, or that are
     * already moving outwards, get proportionally less
     * force. Objects moving in towards the centre get
     * proportionally more force.
     */
    private float peakConcussionForce;

    /**
     * The length of time that the concussion wave is active.
     * As the wave nears this, the forces it applies reduces.
     */
    private float concussionDuration;

    /**
     * This is the peak force for stationary objects in
     * the centre of the convection chimney. Force calculations
     * for this value are the same as for peakConcussionForce.
     */
    private float peakConvectionForce;

    /**
     * The radius of the chimney cylinder in the xz plane.
     */
    private float chimneyRadius;

    /**
     * The maximum height of the chimney.
     */
    private float chimneyHeight;

    /**
     * The length of time the convection chimney is active. Typically
     * this is the longest effect to be in operation, as the heat
     * from the explosion outlives the shock wave and implosion
     * itself.
     */
    private float convectionDuration;

    public ParticleExplosion() {
        this.detonation = new Vector3f();
    }

    public float getTimePassed() {
        return timePassed;
    }

    public void setTimePassed(float timePassed) {
        this.timePassed = timePassed;
    }

    public Vector3f getDetonation() {
        return detonation;
    }

    public void setDetonation(Vector3f detonation) {
        this.detonation = detonation;
    }

    public float getImplosionMaxRadius() {
        return implosionMaxRadius;
    }

    public void setImplosionMaxRadius(float implosionMaxRadius) {
        this.implosionMaxRadius = implosionMaxRadius;
    }

    public float getImplosionMinRadius() {
        return implosionMinRadius;
    }

    public void setImplosionMinRadius(float implosionMinRadius) {
        this.implosionMinRadius = implosionMinRadius;
    }

    public float getImplosionDuration() {
        return implosionDuration;
    }

    public void setImplosionDuration(float implosionDuration) {
        this.implosionDuration = implosionDuration;
    }

    public float getImplosionForce() {
        return implosionForce;
    }

    public void setImplosionForce(float implosionForce) {
        this.implosionForce = implosionForce;
    }

    public float getShockwaveSpeed() {
        return shockwaveSpeed;
    }

    public void setShockwaveSpeed(float shockwaveSpeed) {
        this.shockwaveSpeed = shockwaveSpeed;
    }

    public float getShockwaveThickness() {
        return shockwaveThickness;
    }

    public void setShockwaveThickness(float shockwaveThickness) {
        this.shockwaveThickness = shockwaveThickness;
    }

    public float getPeakConcussionForce() {
        return peakConcussionForce;
    }

    public void setPeakConcussionForce(float peakConcussionForce) {
        this.peakConcussionForce = peakConcussionForce;
    }

    public float getConcussionDuration() {
        return concussionDuration;
    }

    public void setConcussionDuration(float concussionDuration) {
        this.concussionDuration = concussionDuration;
    }

    public float getPeakConvectionForce() {
        return peakConvectionForce;
    }

    public void setPeakConvectionForce(float peakConvectionForce) {
        this.peakConvectionForce = peakConvectionForce;
    }

    public float getChimneyRadius() {
        return chimneyRadius;
    }

    public void setChimneyRadius(float chimneyRadius) {
        this.chimneyRadius = chimneyRadius;
    }

    public float getChimneyHeight() {
        return chimneyHeight;
    }

    public void setChimneyHeight(float chimneyHeight) {
        this.chimneyHeight = chimneyHeight;
    }

    public float getConvectionDuration() {
        return convectionDuration;
    }

    public void setConvectionDuration(float convectionDuration) {
        this.convectionDuration = convectionDuration;
    }

    @Override
    public void updateForce(Particle particle, float duration) {

        // Calculate the vector from the detonation point to the particle's position.
        Vector3f detonationToParticle = particle.getPosition().sub(detonation);

        // Calculate the distance from the particle to the detonation point.
        float distance = detonationToParticle.length();

        // Check if the particle is within the implosion range.
        if (distance >= implosionMinRadius && distance <= implosionMaxRadius) {
            // Calculate and apply the implosion force.
            float implosionFactor = 1.0f - (distance - implosionMinRadius) / (implosionMaxRadius - implosionMinRadius);
            implosionFactor *= (1.0f - timePassed / implosionDuration);
            Vector3f implosionForceVec = detonationToParticle.normalize().mul(implosionForce * implosionFactor);
            particle.addForce(implosionForceVec.mul(duration));
        }

        // Check if the particle is within the shockwave range.
        if (distance <= shockwaveThickness) {
            // Calculate and apply the shockwave force.
            float shockwaveFactor = (shockwaveThickness - distance) / shockwaveThickness;
            Vector3f shockwaveForce = detonationToParticle.normalize().mul(shockwaveSpeed * shockwaveFactor);
            particle.addForce(shockwaveForce.mul(duration));
        }

        // Check if the particle is within the concussion duration.
        if (timePassed <= concussionDuration) {
            // Calculate and apply the concussion force based on distance and time.
            float concussionFactor = 1.0f - (timePassed / concussionDuration);
            Vector3f concussionForce = detonationToParticle.normalize().mul(peakConcussionForce * concussionFactor);
            particle.addForce(concussionForce.mul(duration));
        }

        // Check if the particle is within the convection chimney.
        if (particle.getPosition().getX() >= detonation.getX() - chimneyRadius &&
                particle.getPosition().getX() <= detonation.getX() + chimneyRadius &&
                particle.getPosition().getY() >= detonation.getY() &&
                particle.getPosition().getY() <= detonation.getY() + chimneyHeight &&
                particle.getPosition().getZ() >= detonation.getZ() - chimneyRadius &&
                particle.getPosition().getZ() <= detonation.getZ() + chimneyRadius) {

            // Calculate and apply the convection force.
            float convectionFactor = 1.0f - (particle.getPosition().getY() - detonation.getY()) / chimneyHeight;
            convectionFactor *= (1.0f - timePassed / convectionDuration);
            Vector3f convectionForce = new Vector3f(0.0f, peakConvectionForce * convectionFactor, 0.0f);
            particle.addForce(convectionForce.mul(duration));
        }

    }

}
