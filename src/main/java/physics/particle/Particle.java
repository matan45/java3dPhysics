package physics.particle;

import math.Vector3f;

import java.util.Objects;

public class Particle {
    private float inverseMass;
    private float damping;
    private Vector3f position;
    private Vector3f velocity;
    private Vector3f forceAccumulate;
    private Vector3f acceleration;

    public Particle() {
        this.position = new Vector3f();
        this.velocity = new Vector3f();
        this.forceAccumulate = new Vector3f();
        this.acceleration = new Vector3f();
    }

    public float getMass() {
        if (inverseMass == 0) {
            return Float.MAX_VALUE;
        } else {
            return (1.0f) / inverseMass;
        }
    }

    public float getInverseMass() {
        return inverseMass;
    }

    public void setMass(float mass) {
        assert (mass != 0);
        inverseMass = (1.0f) / mass;
    }

    public void setInverseMass(float inverseMass) {
        this.inverseMass = inverseMass;
    }

    public float getDamping() {
        return damping;
    }

    public void setDamping(float damping) {
        this.damping = damping;
    }

    public Vector3f getPosition() {
        return position;
    }

    public void setPosition(Vector3f position) {
        this.position = position;
    }

    public Vector3f getVelocity() {
        return velocity;
    }

    public void setVelocity(Vector3f velocity) {
        this.velocity = velocity;
    }

    public Vector3f getForceAccumulate() {
        return forceAccumulate;
    }

    public void setForceAccumulate(Vector3f forceAccumulate) {
        this.forceAccumulate = forceAccumulate;
    }

    public Vector3f getAcceleration() {
        return acceleration;
    }

    public void setAcceleration(Vector3f acceleration) {
        this.acceleration = acceleration;
    }

    public void addForce(Vector3f force) {
        forceAccumulate = forceAccumulate.add(force);
    }

    public boolean isFiniteMass() {
        return inverseMass >= 0.0f;
    }

    public void clearAccumulator() {
        forceAccumulate.clear();
    }

    public void integrate(float duration) {
        // We don't integrate things with zero mass.
        if (inverseMass <= 0.0f) return;

        assert (duration > 0.0);

        // Update linear position.
        position = position.add(velocity.mul(duration));
        // Work out the acceleration from the force
        Vector3f resultingAcc = new Vector3f(acceleration);
        resultingAcc = resultingAcc.add(forceAccumulate.mul(inverseMass));

        // Update linear velocity from the acceleration.
        velocity = resultingAcc.add(resultingAcc.mul(duration));
        // Impose drag.
        velocity = velocity.mul((float) Math.pow(damping, duration));

        // Clear the forces.
        clearAccumulator();
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Particle particle = (Particle) o;
        return Float.compare(inverseMass, particle.inverseMass) == 0 && Float.compare(damping, particle.damping) == 0 && Objects.equals(position, particle.position) && Objects.equals(velocity, particle.velocity) && Objects.equals(forceAccumulate, particle.forceAccumulate) && Objects.equals(acceleration, particle.acceleration);
    }

    @Override
    public int hashCode() {
        return Objects.hash(inverseMass, damping, position, velocity, forceAccumulate, acceleration);
    }
}
