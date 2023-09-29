package physics.rigidBody;

import math.Matrix4f;
import math.Quaternion;
import math.Vector3f;

public class RigidBody {
    /**
     * Holds the inverse of the mass of the rigid body. It
     * is more useful to hold the inverse mass because
     * integration is simpler, and because in real time
     * simulation it is more useful to have bodies with
     * infinite mass (immovable) than zero mass
     * (completely unstable in numerical simulation).
     */
    private float inverseMass;

    /**
     * Holds the inverse of the body's inertia tensor. The
     * inertia tensor provided must not be degenerate
     * (that would mean the body had zero inertia for
     * spinning along one axis). As long as the tensor is
     * finite, it will be invertible. The inverse tensor
     * is used for similar reasons to the use of inverse
     * mass.
     * <p>
     * The inertia tensor, unlike the other variables that
     * define a rigid body, is given in body space.
     */
    private Matrix4f inverseInertiaTensor;

    /**
     * Holds the amount of damping applied to linear
     * motion.  Damping is required to remove energy added
     * through numerical instability in the integrator.
     */
    private float linearDamping;

    /**
     * Holds the amount of damping applied to angular
     * motion.  Damping is required to remove energy added
     * through numerical instability in the integrator.
     */
    private float angularDamping;

    /**
     * Holds the linear position of the rigid body in
     * world space.
     */
    private Vector3f position;

    /**
     * Holds the angular orientation of the rigid body in
     * world space.
     */
    private Quaternion orientation;

    /**
     * Holds the linear velocity of the rigid body in
     * world space.
     */
    private Vector3f velocity;

    /**
     * Holds the angular velocity, or rotation, or the
     * rigid body in world space.
     */
    private Vector3f rotation;

    /**
     * Holds the inverse inertia tensor of the body in world
     * space. The inverse inertia tensor member is specified in
     * the body's local space.
     */
    private Matrix4f inverseInertiaTensorWorld;

    /**
     * Holds the amount of motion of the body. This is a recency
     * weighted mean that can be used to put a body to sleep.
     */
    private float motion;

    /**
     * A body can be put to sleep to avoid it being updated
     * by the integration functions or affected by collisions
     * with the world.
     */
    private boolean isAwake;

    /**
     * Some bodies may never be allowed to fall asleep.
     * User controlled bodies, for example, should be
     * always awake.
     */
    private boolean canSleep;

    /**
     * Holds a transform matrix for converting body space into
     * world space and vice versa. This can be achieved by calling
     * the getPointIn*Space functions.
     */
    private Matrix4f transformMatrix;

    /**
     * Holds the accumulated force to be applied at the next
     * integration step.
     */
    private Vector3f forceAccum;

    /**
     * Holds the accumulated torque to be applied at the next
     * integration step.
     */
    private Vector3f torqueAccum;

    /**
     * Holds the acceleration of the rigid body.  This value
     * can be used to set acceleration due to gravity (its primary
     * use), or any other constant acceleration.
     */
    private Vector3f acceleration;

    /**
     * Holds the linear acceleration of the rigid body, for the
     * previous frame.
     */
    private Vector3f lastFrameAcceleration;

    public RigidBody() {
        this.acceleration = new Vector3f();
        this.forceAccum = new Vector3f();
        this.lastFrameAcceleration = new Vector3f();
        this.torqueAccum = new Vector3f();
        this.inverseInertiaTensor = new Matrix4f();
        this.position = new Vector3f();
        this.orientation = new Quaternion();
        this.velocity = new Vector3f();
        this.rotation = new Vector3f();
        this.inverseInertiaTensorWorld = new Matrix4f();
        this.transformMatrix = new Matrix4f();
        this.acceleration = new Vector3f();
    }

    public float getInverseMass() {
        return inverseMass;
    }

    public void setInverseMass(float inverseMass) {
        this.inverseMass = inverseMass;
    }

    public Matrix4f getInverseInertiaTensor() {
        return inverseInertiaTensor;
    }

    public void setInverseInertiaTensor(Matrix4f inverseInertiaTensor) {
        this.inverseInertiaTensor = inverseInertiaTensor;
    }

    public float getLinearDamping() {
        return linearDamping;
    }

    public void setLinearDamping(float linearDamping) {
        this.linearDamping = linearDamping;
    }

    public float getAngularDamping() {
        return angularDamping;
    }

    public void setAngularDamping(float angularDamping) {
        this.angularDamping = angularDamping;
    }

    public Vector3f getPosition() {
        return position;
    }

    public void setPosition(Vector3f position) {
        this.position = position;
    }

    public Quaternion getOrientation() {
        return orientation;
    }

    public void setOrientation(Quaternion orientation) {
        this.orientation = orientation;
    }

    public Vector3f getVelocity() {
        return velocity;
    }

    public void setVelocity(Vector3f velocity) {
        this.velocity = velocity;
    }

    public Vector3f getRotation() {
        return rotation;
    }

    public void setRotation(Vector3f rotation) {
        this.rotation = rotation;
    }

    public Matrix4f getInverseInertiaTensorWorld() {
        return inverseInertiaTensorWorld;
    }

    public void setInverseInertiaTensorWorld(Matrix4f inverseInertiaTensorWorld) {
        this.inverseInertiaTensorWorld = inverseInertiaTensorWorld;
    }

    public float getMotion() {
        return motion;
    }

    public void setMotion(float motion) {
        this.motion = motion;
    }

    public boolean isAwake() {
        return isAwake;
    }

    public void setAwake(boolean awake) {
        isAwake = awake;
    }

    public boolean isCanSleep() {
        return canSleep;
    }

    public void setCanSleep(boolean canSleep) {
        this.canSleep = canSleep;
    }

    public Matrix4f getTransformMatrix() {
        return transformMatrix;
    }

    public void setTransformMatrix(Matrix4f transformMatrix) {
        this.transformMatrix = transformMatrix;
    }

    public Vector3f getForceAccum() {
        return forceAccum;
    }

    public void setForceAccum(Vector3f forceAccum) {
        this.forceAccum = forceAccum;
    }

    public Vector3f getTorqueAccum() {
        return torqueAccum;
    }

    public void setTorqueAccum(Vector3f torqueAccum) {
        this.torqueAccum = torqueAccum;
    }

    public Vector3f getAcceleration() {
        return acceleration;
    }

    public void setAcceleration(Vector3f acceleration) {
        this.acceleration = acceleration;
    }

    public Vector3f getLastFrameAcceleration() {
        return lastFrameAcceleration;
    }

    public void setLastFrameAcceleration(Vector3f lastFrameAcceleration) {
        this.lastFrameAcceleration = lastFrameAcceleration;
    }

    public boolean isFiniteMass() {
        return inverseMass >= 0.0f;
    }
}
