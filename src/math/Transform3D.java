package math;

import java.util.Objects;

public class Transform3D {

    // Orientation
    private final Quaternionf orientation;

    // Position
    private final Vector3f position;

    // Constructor
    public Transform3D() {
        orientation = new Quaternionf().identity();
        position = new Vector3f();
    }

    // Constructor with arguments
    public Transform3D(Vector3f position, Matrix3f orientation) {
        this.orientation = new Quaternionf(orientation);
        this.position = new Vector3f(position);
    }

    // Constructor with arguments
    public Transform3D(Vector3f position, Quaternionf orientation) {
        this.orientation = new Quaternionf(orientation);
        this.position = new Vector3f(position);
    }

    // Copy-constructor
    public Transform3D(Transform3D Transform3D) {
        orientation = new Quaternionf(Transform3D.orientation);
        position = new Vector3f(Transform3D.position);
    }

    // Return the rotation matrix
    public Quaternionf getOrientation() {
        return orientation;
    }

    // Return the position of the Transform3D
    public Vector3f getPosition() {
        return position;
    }

    // Set the Transform3D from an OpenGL Transform3D matrix
    public Transform3D fromOpenGL(float[] openglMatrix) {
        Matrix3f matrix = new Matrix3f(
                openglMatrix[0], openglMatrix[4], openglMatrix[8],
                openglMatrix[1], openglMatrix[5], openglMatrix[9],
                openglMatrix[2], openglMatrix[6], openglMatrix[10]);
        orientation.fromMatrix(matrix);
        position.set(openglMatrix[12], openglMatrix[13], openglMatrix[14]);
        return this;
    }

    // Set the Transform3D to the identity Transform3D
    public Transform3D identity() {
        position.zero();
        orientation.identity();
        return this;
    }

    // Return the inverse of the Transform3D
    public Transform3D inverse() {
        orientation.inverse();
        position.invert();
        Matrix3f invMatrix = orientation.getMatrix(new Matrix3f());
        Vector3f invPosition = new Vector3f();
        invMatrix.multiply(position, invPosition);
        position.set(invPosition);
        return this;
    }

    // Operator of multiplication of a Transform3D with another one
    public Transform3D multiply(Transform3D Transform3D) {
        Matrix3f matrix = orientation.getMatrix(new Matrix3f());
        orientation.multiply(Transform3D.orientation);
        Vector3f newPosition = new Vector3f();
        matrix.multiply(Transform3D.position, newPosition);
        position.add(newPosition);
        return this;
    }

    // Assignment operator
    public Transform3D set(Transform3D Transform3D) {
        orientation.set(Transform3D.orientation);
        position.set(Transform3D.position);
        return this;
    }

    // Set the rotation matrix of the Transform3D
    public Transform3D setOrientation(Quaternionf orientation) {
        this.orientation.set(orientation);
        return this;
    }

    // Set the origin of the Transform3D
    public Transform3D setPosition(Vector3f position) {
        this.position.set(position);
        return this;
    }

    // Get the OpenGL matrix of the Transform3D
    public float[] getOpenGLMatrix(float[] openglMatrix) {
        Matrix3f matrix = new Matrix3f();
        orientation.getMatrix(matrix);
        openglMatrix[0] = matrix.m00;
        openglMatrix[1] = matrix.m10;
        openglMatrix[2] = matrix.m20;
        openglMatrix[3] = 0.0f;
        openglMatrix[4] = matrix.m01;
        openglMatrix[5] = matrix.m11;
        openglMatrix[6] = matrix.m21;
        openglMatrix[7] = 0.0f;
        openglMatrix[8] = matrix.m02;
        openglMatrix[9] = matrix.m12;
        openglMatrix[10] = matrix.m22;
        openglMatrix[11] = 0.0f;
        openglMatrix[12] = position.x;
        openglMatrix[13] = position.y;
        openglMatrix[14] = position.z;
        openglMatrix[15] = 1.0f;
        return openglMatrix;
    }

    // Return the Transform3Ded vector
    public Vector3f multiply(Vector3f vector, Vector3f vectorOut) {
        Matrix3f matrix = orientation.getMatrix(new Matrix3f());
        return matrix.multiply(vector, vectorOut).add(position);
    }

    // Return an interpolated Transform3D
    public static Transform3D Interpolate(Transform3D oldTransform3D, Transform3D newTransform3D, float interpolationFactor, Transform3D outTransform3D) {
        assert (interpolationFactor >= 0.0f && interpolationFactor <= 1.0f);
        Quaternionf interOrientation = new Quaternionf();
        Vector3f interPosition = new Vector3f();
        Quaternionf.Slerp(oldTransform3D.orientation, newTransform3D.orientation, interpolationFactor, interOrientation);
        Vector3f.lerp(oldTransform3D.position, newTransform3D.position, interpolationFactor, interPosition);
        outTransform3D.setOrientation(interOrientation);
        outTransform3D.setPosition(interPosition);
        return outTransform3D;
    }

    @Override
    public int hashCode() {
        int hash = 3;
        hash = 71 * hash + Objects.hashCode(this.position);
        hash = 71 * hash + Objects.hashCode(this.orientation);
        return hash;
    }

    @Override
    public boolean equals(Object obj) {

        if (obj == null) {
            return false;
        }
        if (getClass() != obj.getClass()) {
            return false;
        }
        final Transform3D other = (Transform3D) obj;
        if (!Objects.equals(this.position, other.position)) {
            return false;
        }
        return Objects.equals(this.orientation, other.orientation);
    }

    @Override
    public String toString() {
        return "(position= " + position + ", orientation= " + orientation + ")";
    }

}