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

    // ... Other Explosion code as before ...


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

        // Calculate the distance from the detonation point.
        float distance = particle.getPosition().distance(detonation);

        // Apply the implosion force, if within range.
        if (distance < implosionMaxRadius && distance > implosionMinRadius) {
            particle.addForce(new Vector3f(0, -implosionForce, 0));
        }

        // Apply the shockwave force, if within range.
        if (distance < shockwaveThickness) {
            // Calculate the force based on the distance from the wavefront.
            float force = peakConcussionForce * (shockwaveThickness - distance) / shockwaveThickness;

            // Calculate the direction of the force.
            Vector3f forceDirection = particle.getPosition().sub(detonation).normalize();

            // Apply the force.
            particle.addForce(forceDirection.mul(force));
        }

        // Apply the convection chimney force, if within range.
        if (distance < chimneyRadius && distance > 0) {
            // Calculate the force based on the distance from the centre of the chimney.
            float force = peakConvectionForce * (chimneyRadius - distance) / chimneyRadius;

            // Calculate the direction of the force.
            Vector3f forceDirection = particle.getPosition().sub(detonation).normalize();

            // Apply the force.
            particle.addForce(forceDirection.mul(force));
        }

    }

}
