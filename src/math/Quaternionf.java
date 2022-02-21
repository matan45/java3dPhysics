package math;

public class Quaternionf {

    // Component x
    float x;

    // Component y
    float y;

    // Component z
    float z;

    // Component w
    float w;

    // Constructor
    public Quaternionf() {
        zero();
    }

    // Constructor with arguments
    public Quaternionf(float x, float y, float z, float w) {
        set(x, y, z, w);
    }

    // Create a unit Quaternionf from a rotation matrix
    public Quaternionf(Matrix3f matrix) {
        fromMatrix(matrix);
    }

    // Copy-constructor
    public Quaternionf(Quaternionf Quaternionf) {
        set(Quaternionf);
    }

    // Constructor with the component w and the vector v=(x y z)
    public Quaternionf(Vector3f vector, float w) {
        set(vector, w);
    }

    // Scalar product between two Quaternionfs
    public float dot(Quaternionf Quaternionf) {
        return x * Quaternionf.x + y * Quaternionf.y + z * Quaternionf.z + w * Quaternionf.w;
    }

    public float getW() {
        return w;
    }

    public float getX() {
        return x;
    }

    public float getY() {
        return y;
    }

    public float getZ() {
        return z;
    }

    // Return the length of the Quaternionf (public )
    public float length() {
        return Mathematics.Sqrt(x * x + y * y + z * z + w * w);
    }

    // Return the square of the length of the Quaternionf
    public float lengthSquare() {
        return x * x + y * y + z * z + w * w;
    }

    // Overloaded operator for addition with assignment
    public Quaternionf add(Quaternionf Quaternionf) {
        x += Quaternionf.x;
        y += Quaternionf.y;
        z += Quaternionf.z;
        w += Quaternionf.w;
        return this;
    }

    // Return the conjugate of the Quaternionf (public )
    public Quaternionf conjugate() {
        x = -x;
        y = -y;
        z = -z;
        return this;
    }

    public final Quaternionf fromMatrix(Matrix3f matrix) {

        // Get the trace of the matrix
        float r, s, trace = matrix.getTrace();

        if (trace < 0.0f) {
            if (matrix.m11 > matrix.m00) {
                if (matrix.m22 > matrix.m11) {
                    r = Mathematics.Sqrt(matrix.m22 - matrix.m00 - matrix.m11 + 1.0f);
                    s = 0.5f / r;

                    // Compute the Quaternionf
                    x = (matrix.m20 + matrix.m02) * s;
                    y = (matrix.m12 + matrix.m21) * s;
                    z = 0.5f * r;
                    w = (matrix.m10 - matrix.m01) * s;
                } else {
                    r = Mathematics.Sqrt(matrix.m11 - matrix.m22 - matrix.m00 + 1.0f);
                    s = 0.5f / r;

                    // Compute the Quaternionf
                    x = (matrix.m01 + matrix.m10) * s;
                    y = 0.5f * r;
                    z = (matrix.m12 + matrix.m21) * s;
                    w = (matrix.m02 - matrix.m20) * s;
                }
            } else if (matrix.m22 > matrix.m00) {
                r = Mathematics.Sqrt(matrix.m22 - matrix.m00 - matrix.m11 + 1.0f);
                s = 0.5f / r;

                // Compute the Quaternionf
                x = (matrix.m20 + matrix.m02) * s;
                y = (matrix.m12 + matrix.m21) * s;
                z = 0.5f * r;
                w = (matrix.m10 - matrix.m01) * s;
            } else {
                r = Mathematics.Sqrt(matrix.m00 - matrix.m11 - matrix.m22 + 1.0f);
                s = 0.5f / r;

                // Compute the Quaternionf
                x = 0.5f * r;
                y = (matrix.m01 + matrix.m10) * s;
                z = (matrix.m20 - matrix.m02) * s;
                w = (matrix.m21 - matrix.m12) * s;
            }
        } else {
            r = Mathematics.Sqrt(trace + 1.0f);
            s = 0.5f / r;

            // Compute the Quaternionf
            x = (matrix.m21 - matrix.m12) * s;
            y = (matrix.m02 - matrix.m20) * s;
            z = (matrix.m10 - matrix.m01) * s;
            w = 0.5f * r;
        }

        return this;
    }

    // Set to the identity Quaternionf
    public Quaternionf identity() {
        x = 0.0f;
        y = 0.0f;
        z = 0.0f;
        w = 1.0f;
        return this;
    }

    // Inverse the Quaternionf
    public Quaternionf inverse() {

        // Get the square length of the Quaternionf
        float lenSq = lengthSquare();
        assert (lenSq > Mathematics.MACHINE_EPSILON);

        // Compute and return the inverse Quaternionf
        x /= -lenSq;
        y /= -lenSq;
        z /= -lenSq;
        w /= lenSq;
        return this;
    }

    // Overloaded operator for the multiplication with a constant
    public Quaternionf multiply(float number) {
        x *= number;
        y *= number;
        z *= number;
        w *= number;
        return this;
    }

    // Overloaded operator for the multiplication of two Quaternionfs
    public Quaternionf multiply(Quaternionf Quaternionf) {
        Vector3f q1V = new Vector3f();
        getVectorV(q1V);
        Vector3f q2V = new Vector3f();
        Quaternionf.getVectorV(q2V);
        Vector3f newVector = new Vector3f(q2V).multiply(w)
                .add(new Vector3f(q1V).multiply(Quaternionf.w))
                .add(new Vector3f(q1V).cross(q2V));
        return set(
                newVector.getX(), newVector.getY(), newVector.getZ(),
                w * Quaternionf.w - q1V.dot(q2V));
    }

    // Normalize the Quaternionf
    public Quaternionf normalize() {
        float len = length();
        assert (len > Mathematics.MACHINE_EPSILON);
        float lenInv = 1.0f / len;
        x *= lenInv;
        y *= lenInv;
        z *= lenInv;
        w *= lenInv;
        return this;
    }

    // Set all the values
    public final Quaternionf set(float x, float y, float z, float w) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.w = w;
        return this;
    }

    public final Quaternionf set(Quaternionf Quaternionf) {
        x = Quaternionf.x;
        y = Quaternionf.y;
        z = Quaternionf.z;
        w = Quaternionf.w;
        return this;
    }

    public final Quaternionf set(Vector3f vector, float w) {
        x = vector.x;
        y = vector.y;
        z = vector.z;
        this.w = w;
        return this;
    }

    public Quaternionf setW(float w) {
        this.w = w;
        return this;
    }

    public Quaternionf setX(float x) {
        this.x = x;
        return this;
    }

    public Quaternionf setY(float y) {
        this.y = y;
        return this;
    }

    public Quaternionf setZ(float z) {
        this.z = z;
        return this;
    }

    // Overloaded operator for substraction with assignment
    public Quaternionf subtract(Quaternionf Quaternionf) {
        x -= Quaternionf.x;
        y -= Quaternionf.y;
        z -= Quaternionf.z;
        w -= Quaternionf.w;
        return this;
    }

    // Set the Quaternionf to zero
    public final Quaternionf zero() {
        x = 0.0f;
        y = 0.0f;
        z = 0.0f;
        w = 0.0f;
        return this;
    }

    // Return the orientation matrix corresponding to this Quaternionf
    public Matrix3f getMatrix(Matrix3f matrix) {

        float nQ = x * x + y * y + z * z + w * w;
        float s = 0.0f;

        if (nQ > 0.0f) {
            s = 2.0f / nQ;
        }

        // Computations used for optimization (less multiplications)
        float xs = x * s;
        float ys = y * s;
        float zs = z * s;
        float wxs = w * xs;
        float wys = w * ys;
        float wzs = w * zs;
        float xxs = x * xs;
        float xys = x * ys;
        float xzs = x * zs;
        float yys = y * ys;
        float yzs = y * zs;
        float zzs = z * zs;

        // Create the matrix corresponding to the Quaternionf
        return matrix.set(1.0f - yys - zzs, xys - wzs, xzs + wys,
                xys + wzs, 1.0f - xxs - zzs, yzs - wxs,
                xzs - wys, yzs + wxs, 1.0f - xxs - yys);
    }

    // Compute the rotation angle (in radians) and the rotation axis
    // This method is used to get the rotation angle (in radian) and the unit
    // rotation axis of an orientation Quaternionf.
    public Vector3f getRotationAngleAxis(Vector3f axis, float[] angle) {

        Quaternionf Quaternionf;

        // If the Quaternionf is unit
        if (length() == 1.0) {
            Quaternionf = this;
        } else {
            // We compute the unit Quaternionf
            Quaternionf = new Quaternionf(this).normalize();
        }

        // Compute the roation angle
        angle[0] = Mathematics.ArcCos(Quaternionf.w) * 2.0f;

        // Compute the 3D rotation axis
        Vector3f rotationAxis = new Vector3f(Quaternionf.x, Quaternionf.y, Quaternionf.z);

        // Normalize the rotation axis
        rotationAxis.normalize();

        // Set the rotation axis values
        return axis.set(rotationAxis);
    }

    // Return the vector v=(x y z) of the Quaternionf
    public Vector3f getVectorV(Vector3f vector) {
        return vector.set(x, y, z);
    }

    // Overloaded operator for the multiplication with a vector.
    // This methods rotates a point given the rotation of a Quaternionf.
    public Vector3f multiply(Vector3f vector, Vector3f vectorOut) {
        Quaternionf c = new Quaternionf(this).conjugate();
        Quaternionf p = new Quaternionf(vector.x, vector.y, vector.z, 0.0f);
        new Quaternionf(this).multiply(p).multiply(c).getVectorV(vectorOut);
        return vectorOut;
    }

    // Compute the spherical linear interpolation between two Quaternionfs.
    // The t argument has to be such that 0 <= t <= 1. This method is static.
    public static void Slerp(Quaternionf oldQuaternionf, Quaternionf newQuaternionf2, float t, Quaternionf QuaternionfOut) {

        assert (t >= 0.0f && t <= 1.0f);

        float invert = 1.0f;
        Quaternionf tempQ2 = new Quaternionf(newQuaternionf2);

        // Compute cos(theta) using the Quaternionf scalar product
        float cosineTheta = oldQuaternionf.dot(newQuaternionf2);

        // Take care of the sign of cosineTheta
        if (cosineTheta < 0.0f) {
            cosineTheta = -cosineTheta;
            invert = -1.0f;
        }

        // Because of precision, if cos(theta) is nearly 1,
        // therefore theta is nearly 0 and we can write
        // sin((1-t)*theta) as (1-t) and sin(t*theta) as t
        float epsilon = 0.00001f;
        if (1 - cosineTheta < epsilon) {
            QuaternionfOut.set(oldQuaternionf).multiply(1.0f - t).add(tempQ2.multiply(t * invert));
            return;
        }

        // Compute the theta angle
        float theta = Mathematics.ArcCos(cosineTheta);

        // Compute sin(theta)
        float sineTheta = Mathematics.Sin(theta);

        // Compute the two coefficients that are in the spherical linear interpolation formula
        float coeff1 = Mathematics.Sin((1.0f - t) * theta) / sineTheta;
        float coeff2 = Mathematics.Sin(t * theta) / sineTheta * invert;

        // Compute and return the interpolated Quaternionf
        QuaternionfOut.set(oldQuaternionf).multiply(coeff1).add(tempQ2.multiply(coeff2));
    }

    @Override
    public int hashCode() {
        int hash = 7;
        hash = 59 * hash + Float.floatToIntBits(this.x);
        hash = 59 * hash + Float.floatToIntBits(this.y);
        hash = 59 * hash + Float.floatToIntBits(this.z);
        hash = 59 * hash + Float.floatToIntBits(this.w);
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
        final Quaternionf other = (Quaternionf) obj;
        if (Float.floatToIntBits(this.x) != Float.floatToIntBits(other.x)) {
            return false;
        }
        if (Float.floatToIntBits(this.y) != Float.floatToIntBits(other.y)) {
            return false;
        }
        if (Float.floatToIntBits(this.z) != Float.floatToIntBits(other.z)) {
            return false;
        }
        return Float.floatToIntBits(this.w) == Float.floatToIntBits(other.w);
    }

    @Override
    public String toString() {
        return "(w= " + w + ", x= " + x + ", y= " + y + ", z= " + z + ")";
    }

}