package physics.rigidBody;

import collisionDetection.narrowPhase.Shape;
import math.Matrix3f;
import math.Matrix4f;
import math.Quaternion;
import math.Vector3f;

import static math.Const.SLEEP_EPSILON;

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
    private Matrix3f inverseInertiaTensor;

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
    private Matrix3f inverseInertiaTensorWorld;

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

    private Shape colliderShape;

    public RigidBody() {
        this.acceleration = new Vector3f();
        this.forceAccum = new Vector3f();
        this.lastFrameAcceleration = new Vector3f();
        this.torqueAccum = new Vector3f();
        this.inverseInertiaTensor = new Matrix3f();
        this.position = new Vector3f();
        this.orientation = new Quaternion();
        this.velocity = new Vector3f();
        this.rotation = new Vector3f();
        this.inverseInertiaTensorWorld = new Matrix3f();
        this.transformMatrix = new Matrix4f();
        this.acceleration = new Vector3f();
    }

    public Shape getColliderShape() {
        return colliderShape;
    }

    public void setColliderShape(Shape colliderShape) {
        this.colliderShape = colliderShape;
    }

    public float getInverseMass() {
        return inverseMass;
    }

    public void setInverseMass(float inverseMass) {
        this.inverseMass = inverseMass;
    }

    public Matrix3f getInverseInertiaTensor() {
        return inverseInertiaTensor;
    }

    public void setInverseInertiaTensor(Matrix3f inverseInertiaTensor) {
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

    public Matrix3f getInverseInertiaTensorWorld() {
        return inverseInertiaTensorWorld;
    }

    public void setInverseInertiaTensorWorld(Matrix3f inverseInertiaTensorWorld) {
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
        
        if (!canSleep && !isAwake) setAwake(true);
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

    public boolean hasCollider() {
        return colliderShape != null;
    }

    public void addForce(Vector3f force) {
        forceAccum = forceAccum.add(force);
        isAwake = true;
    }

    public Vector3f getPointInWorldSpace(Vector3f point) {
        return transformMatrix.transform(point);
    }

    public void addTorque(Vector3f torque) {
        torqueAccum = torqueAccum.add(torque);
        isAwake = true;
    }

    public void addForceAtBodyPoint(Vector3f force, Vector3f point) {
        // Convert to coordinates relative to center of mass.
        Vector3f pt = getPointInWorldSpace(point);
        addForceAtPoint(force, pt);
    }

    public void addForceAtPoint(Vector3f force, Vector3f point) {
        // Convert to coordinates relative to center of mass.
        Vector3f pt = point.sub(position);

        forceAccum = forceAccum.add(force);
        torqueAccum = torqueAccum.add(pt.cross(force));

        isAwake = true;
    }

    private void clearAccumulators() {
        forceAccum.clear();
        torqueAccum.clear();
    }

    public void integrate(float duration) {
        if (!isAwake) return;

        // Calculate linear acceleration from force inputs.
        lastFrameAcceleration = acceleration.add(acceleration.mul(forceAccum.mul(inverseMass)));

        // Calculate angular acceleration from torque inputs.
        Vector3f angularAcceleration = inverseInertiaTensorWorld.transform(torqueAccum);

        // Adjust velocities
        // Update linear velocity from both acceleration and impulse.
        velocity = lastFrameAcceleration.add(lastFrameAcceleration.mul(duration));

        // Update angular velocity from both acceleration and impulse.
        rotation = rotation.add(angularAcceleration.mul(duration));

        // Impose drag.
        velocity = velocity.mul((float) Math.pow(linearDamping, duration));
        rotation = rotation.mul((float) Math.pow(angularDamping, duration));

        // Adjust positions
        // Update linear position.
        position = position.add(velocity.mul(duration));

        // Update angular position.
        orientation = orientation.add(rotation.mul(duration));

        // Normalise the orientation, and update the matrices with the new
        // position and orientation
        calculateDerivedData();

        // Clear accumulators.
        clearAccumulators();

        // Update the kinetic energy store, and possibly put the body to
        // sleep.
        if (canSleep) {
            float currentMotion = velocity.dot(velocity) +
                    rotation.dot(rotation);

            float bias = (float) Math.pow(0.5, duration);
            motion = bias * motion + (1 - bias) * currentMotion;

            if (motion < SLEEP_EPSILON) setAwake(false);
            else if (motion > 10 * SLEEP_EPSILON) motion = 10 * SLEEP_EPSILON;
        }
    }

    private void calculateTransformMatrix() {
        // Extract the rotation matrix from the quaternion.
        float xx = orientation.x * orientation.x;
        float xy = orientation.x * orientation.y;
        float xz = orientation.x * orientation.z;
        float yy = orientation.y * orientation.y;
        float yz = orientation.y * orientation.z;
        float zz = orientation.z * orientation.z;
        float wx = orientation.w * orientation.x;
        float wy = orientation.w * orientation.y;
        float wz = orientation.w * orientation.z;

        transformMatrix.setM00(1.0f - 2.0f * (yy + zz));
        transformMatrix.setM01(2.0f * (xy - wz));
        transformMatrix.setM02(2.0f * (xz + wy));
        transformMatrix.setM03(0.0f);
        transformMatrix.setM10(2.0f * (xy + wz));
        transformMatrix.setM11(1.0f - 2.0f * (xx + zz));
        transformMatrix.setM12(2.0f * (yz - wx));
        transformMatrix.setM13(0.0f);
        transformMatrix.setM20(2.0f * (xz - wy));
        transformMatrix.setM21(2.0f * (yz + wx));
        transformMatrix.setM22(1.0f - 2.0f * (xx + yy));
        transformMatrix.setM23(0.0f);
        transformMatrix.setM30(position.x);
        transformMatrix.setM31(position.y);
        transformMatrix.setM32(position.z);
        transformMatrix.setM33(1.0f);

    }

    private void transformInertiaTensor() {

        transformMatrix.setM00(1 - 2 * orientation.z * orientation.z -
                2 * orientation.w * orientation.w);
        transformMatrix.setM01(2 * orientation.y * orientation.z -
                2 * orientation.x * orientation.w);
        transformMatrix.setM02(2 * orientation.y * orientation.w +
                2 * orientation.x * orientation.z);
        transformMatrix.setM03(position.x);

        transformMatrix.setM10(2 * orientation.y * orientation.z +
                2 * orientation.x * orientation.w);
        transformMatrix.setM11(1 - 2 * orientation.y * orientation.y -
                2 * orientation.w * orientation.w);
        transformMatrix.setM12(2 * orientation.z * orientation.w -
                2 * orientation.x * orientation.y);
        transformMatrix.setM13(position.y);

        transformMatrix.setM20(2 * orientation.y * orientation.w -
                2 * orientation.x * orientation.z);
        transformMatrix.setM21(2 * orientation.z * orientation.w +
                2 * orientation.x * orientation.y);
        transformMatrix.setM22(1 - 2 * orientation.y * orientation.y -
                2 * orientation.z * orientation.z);
        transformMatrix.setM23(position.z);

    }

    private void calculateDerivedData() {
        orientation = orientation.normalize();

        // Calculate the transform matrix for the body.
        calculateTransformMatrix();

        // Calculate the inertiaTensor in world space.
        transformInertiaTensor();
    }

    public Vector3f getPointInLocalSpace(Vector3f point) {
        return transformMatrix.transformInverse(point);
    }

    public Vector3f getDirectionInWorldSpace(Vector3f direction) {
        return transformMatrix.transformDirection(direction);
    }

}
