package math;

public class Matrix3f {

    // Rows of the matrix (m[row][column])
    float m00;
    float m01;
    float m02;
    float m10;
    float m11;
    float m12;
    float m20;
    float m21;
    float m22;

    // Constructor
    public Matrix3f() {
        zero();
    }

    // Constructor with arguments
    public Matrix3f(float value) {
        set(value, value, value, value, value, value, value, value, value);
    }

    // Constructor with arguments
    public Matrix3f(
            float a1, float a2, float a3,
            float b1, float b2, float b3,
            float c1, float c2, float c3) {
        set(a1, a2, a3, b1, b2, b3, c1, c2, c3);
    }

    // Copy-constructor
    public Matrix3f(Matrix3f matrix) {
        set(matrix);
    }

    // Return the determinant of the matrix
    public float getDeterminant() {
        return (m00 * (m11 * m22 - m21 * m12)
                - m01 * (m10 * m22 - m20 * m12)
                + m02 * (m10 * m21 - m20 * m11));
    }

    // Return the trace of the matrix
    public float getTrace() {
        return m00 + m11 + m22;
    }

    // Return the matrix with absolute values
    public Matrix3f abs() {
        m00 = Math.abs(m00);
        m01 = Math.abs(m01);
        m02 = Math.abs(m02);
        m10 = Math.abs(m10);
        m11 = Math.abs(m11);
        m12 = Math.abs(m12);
        m20 = Math.abs(m20);
        m21 = Math.abs(m21);
        m22 = Math.abs(m22);
        return this;
    }

    // Overloaded operator for addition with assignment
    public Matrix3f add(Matrix3f matrix) {
        m00 += matrix.m00;
        m01 += matrix.m01;
        m02 += matrix.m02;
        m10 += matrix.m10;
        m11 += matrix.m11;
        m12 += matrix.m12;
        m20 += matrix.m20;
        m21 += matrix.m21;
        m22 += matrix.m22;
        return this;
    }

    // Return a skew-symmetric matrix using a given vector that can be used
    // to compute cross product with another vector using matrix multiplication
    public Matrix3f computeSkewSymmetricMatrixForCrossProduct(Vector3f vector) {
        return set(0.0f, -vector.z, vector.y, vector.z, 0, -vector.x, -vector.y, vector.x, 0.0f);
    }

    // Set the matrix to the identity matrix
    public Matrix3f identity() {
        m00 = 1.0f;
        m01 = 0.0f;
        m02 = 0.0f;
        m10 = 0.0f;
        m11 = 1.0f;
        m12 = 0.0f;
        m20 = 0.0f;
        m21 = 0.0f;
        m22 = 1.0f;
        return this;
    }

    // Return the inverse matrix
    public Matrix3f inverse() {

        // Compute the determinant of the matrix
        float determinant = getDeterminant();

        // Check if the determinant is equal to zero
        assert (Math.abs(determinant) > Mathematics.MACHINE_EPSILON);
        float invDeterminant = 1.0f / determinant;

        set(
                (m11 * m22 - m21 * m12),
                -(m01 * m22 - m21 * m02),
                (m01 * m12 - m02 * m11),
                -(m10 * m22 - m20 * m12),
                (m00 * m22 - m20 * m02),
                -(m00 * m12 - m10 * m02),
                (m10 * m21 - m20 * m11),
                -(m00 * m21 - m20 * m01),
                (m00 * m11 - m01 * m10));

        // Return the inverse matrix
        return multiply(invDeterminant);
    }

    // Overloaded operator for the negative of the matrix
    public Matrix3f invert() {
        m00 = -m00;
        m01 = -m01;
        m02 = -m02;
        m10 = -m10;
        m11 = -m11;
        m12 = -m12;
        m20 = -m20;
        m21 = -m21;
        m22 = -m22;
        return this;
    }

    // Overloaded operator for multiplication with a number with assignment
    public Matrix3f multiply(float number) {
        m00 *= number;
        m01 *= number;
        m02 *= number;
        m10 *= number;
        m11 *= number;
        m12 *= number;
        m20 *= number;
        m21 *= number;
        m22 *= number;
        return this;
    }

    // Overloaded operator for matrix multiplication
    public Matrix3f multiply(Matrix3f matrix) {
        return set(
                m00 * matrix.m00 + m01 * matrix.m10 + m02 * matrix.m20,
                m00 * matrix.m01 + m01 * matrix.m11 + m02 * matrix.m21,
                m00 * matrix.m02 + m01 * matrix.m12 + m02 * matrix.m22,
                m10 * matrix.m00 + m11 * matrix.m10 + m12 * matrix.m20,
                m10 * matrix.m01 + m11 * matrix.m11 + m12 * matrix.m21,
                m10 * matrix.m02 + m11 * matrix.m12 + m12 * matrix.m22,
                m20 * matrix.m00 + m21 * matrix.m10 + m22 * matrix.m20,
                m20 * matrix.m01 + m21 * matrix.m11 + m22 * matrix.m21,
                m20 * matrix.m02 + m21 * matrix.m12 + m22 * matrix.m22);
    }

    // Method to set all the values in the matrix
    public final Matrix3f set(
            float a1, float a2, float a3,
            float b1, float b2, float b3,
            float c1, float c2, float c3) {
        m00 = a1;
        m01 = a2;
        m02 = a3;
        m10 = b1;
        m11 = b2;
        m12 = b3;
        m20 = c1;
        m21 = c2;
        m22 = c3;
        return this;
    }

    public final Matrix3f set(Matrix3f matrix) {
        m00 = matrix.m00;
        m01 = matrix.m01;
        m02 = matrix.m02;
        m10 = matrix.m10;
        m11 = matrix.m11;
        m12 = matrix.m12;
        m20 = matrix.m20;
        m21 = matrix.m21;
        m22 = matrix.m22;
        return this;
    }

    // Overloaded operator for substraction with assignment
    public Matrix3f subtract(Matrix3f matrix) {
        m00 -= matrix.m00;
        m01 -= matrix.m01;
        m02 -= matrix.m02;
        m10 -= matrix.m10;
        m11 -= matrix.m11;
        m12 -= matrix.m12;
        m20 -= matrix.m20;
        m21 -= matrix.m21;
        m22 -= matrix.m22;
        return this;
    }

    // Return the transpose matrix
    public Matrix3f transpose() {
        return set(
                m00, m10, m20,
                m01, m11, m21,
                m02, m12, m22);
    }

    // Set the matrix to zero
    public final void zero() {
        m00 = 0.0f;
        m01 = 0.0f;
        m02 = 0.0f;
        m10 = 0.0f;
        m11 = 0.0f;
        m12 = 0.0f;
        m20 = 0.0f;
        m21 = 0.0f;
        m22 = 0.0f;
    }

    // Return a column
    public Vector3f getColumn(int index) {
        if (index == 0) {
            return new Vector3f(m00, m10, m20);
        } else if (index == 1) {
            return new Vector3f(m01, m11, m21);
        } else if (index == 2) {
            return new Vector3f(m02, m12, m22);
        }
        throw new IllegalArgumentException("Unknown column index: " + index);
    }

    // Return a row
    public Vector3f getRow(int index) {
        if (index == 0) {
            return new Vector3f(m00, m01, m02);
        } else if (index == 1) {
            return new Vector3f(m10, m11, m12);
        } else if (index == 2) {
            return new Vector3f(m20, m21, m22);
        }
        throw new IllegalArgumentException("Unknown column index: " + index);
    }

    // Overloaded operator for multiplication with a vector
    public Vector3f multiply(Vector3f vector, Vector3f vectorOut) {
        return vectorOut.set(
                m00 * vector.x + m01 * vector.y + m02 * vector.z,
                m10 * vector.x + m11 * vector.y + m12 * vector.z,
                m20 * vector.x + m21 * vector.y + m22 * vector.z);
    }

    @Override
    public int hashCode() {
        int hash = 7;
        hash = 71 * hash + Float.floatToIntBits(this.m00);
        hash = 71 * hash + Float.floatToIntBits(this.m01);
        hash = 71 * hash + Float.floatToIntBits(this.m02);
        hash = 71 * hash + Float.floatToIntBits(this.m10);
        hash = 71 * hash + Float.floatToIntBits(this.m11);
        hash = 71 * hash + Float.floatToIntBits(this.m12);
        hash = 71 * hash + Float.floatToIntBits(this.m20);
        hash = 71 * hash + Float.floatToIntBits(this.m21);
        hash = 71 * hash + Float.floatToIntBits(this.m22);
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
        final Matrix3f other = (Matrix3f) obj;
        if (Float.floatToIntBits(this.m00) != Float.floatToIntBits(other.m00)) {
            return false;
        }
        if (Float.floatToIntBits(this.m01) != Float.floatToIntBits(other.m01)) {
            return false;
        }
        if (Float.floatToIntBits(this.m02) != Float.floatToIntBits(other.m02)) {
            return false;
        }
        if (Float.floatToIntBits(this.m10) != Float.floatToIntBits(other.m10)) {
            return false;
        }
        if (Float.floatToIntBits(this.m11) != Float.floatToIntBits(other.m11)) {
            return false;
        }
        if (Float.floatToIntBits(this.m12) != Float.floatToIntBits(other.m12)) {
            return false;
        }
        if (Float.floatToIntBits(this.m20) != Float.floatToIntBits(other.m20)) {
            return false;
        }
        if (Float.floatToIntBits(this.m21) != Float.floatToIntBits(other.m21)) {
            return false;
        }
        return Float.floatToIntBits(this.m22) == Float.floatToIntBits(other.m22);
    }

    @Override
    public String toString() {
        return "(00= " + m00 + ", 01= " + m01 + ", 02= " + m02
                + ", 10= " + m10 + ", 11= " + m11 + ", 12= " + m12
                + ", 20= " + m20 + ", 21= " + m21 + ", 22= " + m22 + ")";
    }

}