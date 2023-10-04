package math;

import java.util.Objects;

import static math.Const.EPSILON;

public class Matrix4f {
    float m00, m01, m02, m03;
    float m10, m11, m12, m13;
    float m20, m21, m22, m23;
    float m30, m31, m32, m33;

    public Matrix4f() {
        setIdentity();
    }

    public Matrix4f(Matrix4f other) {
        m00 = other.m00;
        m01 = other.m01;
        m02 = other.m02;
        m03 = other.m03;
        m10 = other.m10;
        m11 = other.m11;
        m12 = other.m12;
        m13 = other.m13;
        m20 = other.m20;
        m21 = other.m21;
        m22 = other.m22;
        m23 = other.m23;
        m30 = other.m30;
        m31 = other.m31;
        m32 = other.m32;
        m33 = other.m33;
    }

    public Matrix4f setIdentity() {
        m00 = 1.0f;
        m01 = 0.0f;
        m02 = 0.0f;
        m03 = 0.0f;
        m10 = 0.0f;
        m11 = 1.0f;
        m12 = 0.0f;
        m13 = 0.0f;
        m20 = 0.0f;
        m21 = 0.0f;
        m22 = 1.0f;
        m23 = 0.0f;
        m30 = 0.0f;
        m31 = 0.0f;
        m32 = 0.0f;
        m33 = 1.0f;

        return this;
    }

    public void setMatrix(Matrix4f other) {
        m00 = other.m00;
        m01 = other.m01;
        m02 = other.m02;
        m03 = other.m03;
        m10 = other.m10;
        m11 = other.m11;
        m12 = other.m12;
        m13 = other.m13;
        m20 = other.m20;
        m21 = other.m21;
        m22 = other.m22;
        m23 = other.m23;
        m30 = other.m30;
        m31 = other.m31;
        m32 = other.m32;
        m33 = other.m33;
    }

    public Matrix4f add(Matrix4f right) {

        m00 += right.m00;
        m01 += right.m01;
        m02 += right.m02;
        m03 += right.m03;
        m10 += right.m10;
        m11 += right.m11;
        m12 += right.m12;
        m13 += right.m13;
        m20 += right.m20;
        m21 += right.m21;
        m22 += right.m22;
        m23 += right.m23;
        m30 += right.m30;
        m31 += right.m31;
        m32 += right.m32;
        m33 += right.m33;

        return this;
    }

    public Matrix4f sub(Matrix4f right) {

        m00 -= right.m00;
        m01 -= right.m01;
        m02 -= right.m02;
        m03 -= right.m03;
        m10 -= right.m10;
        m11 -= right.m11;
        m12 -= right.m12;
        m13 -= right.m13;
        m20 -= right.m20;
        m21 -= right.m21;
        m22 -= right.m22;
        m23 -= right.m23;
        m30 -= right.m30;
        m31 -= right.m31;
        m32 -= right.m32;
        m33 -= right.m33;

        return this;
    }

    public Matrix4f mul(Matrix4f right) {

        m00 = m00 * right.m00 + m10 * right.m01 + m20 * right.m02 + m30 * right.m03;
        m01 = m01 * right.m00 + m11 * right.m01 + m21 * right.m02 + m31 * right.m03;
        m02 = m02 * right.m00 + m12 * right.m01 + m22 * right.m02 + m32 * right.m03;
        m03 = m03 * right.m00 + m13 * right.m01 + m23 * right.m02 + m33 * right.m03;
        m10 = m00 * right.m10 + m10 * right.m11 + m20 * right.m12 + m30 * right.m13;
        m11 = m01 * right.m10 + m11 * right.m11 + m21 * right.m12 + m31 * right.m13;
        m12 = m02 * right.m10 + m12 * right.m11 + m22 * right.m12 + m32 * right.m13;
        m13 = m03 * right.m10 + m13 * right.m11 + m23 * right.m12 + m33 * right.m13;
        m20 = m00 * right.m20 + m10 * right.m21 + m20 * right.m22 + m30 * right.m23;
        m21 = m01 * right.m20 + m11 * right.m21 + m21 * right.m22 + m31 * right.m23;
        m22 = m02 * right.m20 + m12 * right.m21 + m22 * right.m22 + m32 * right.m23;
        m23 = m03 * right.m20 + m13 * right.m21 + m23 * right.m22 + m33 * right.m23;
        m30 = m00 * right.m30 + m10 * right.m31 + m20 * right.m32 + m30 * right.m33;
        m31 = m01 * right.m30 + m11 * right.m31 + m21 * right.m32 + m31 * right.m33;
        m32 = m02 * right.m30 + m12 * right.m31 + m22 * right.m32 + m32 * right.m33;
        m33 = m03 * right.m30 + m13 * right.m31 + m23 * right.m32 + m33 * right.m33;

        return this;
    }

    public Matrix4f translate(Vector3f vec) {

        m30 += m00 * vec.x + m10 * vec.y + m20 * vec.z;
        m31 += m01 * vec.x + m11 * vec.y + m21 * vec.z;
        m32 += m02 * vec.x + m12 * vec.y + m22 * vec.z;
        m33 += m03 * vec.x + m13 * vec.y + m23 * vec.z;

        return this;
    }

    public Vector3f transform(Vector3f vector) {
        float x = m00 * vector.x + m01 * vector.y + m02 * vector.z;
        float y = m10 * vector.x + m11 * vector.y + m12 * vector.z;
        float z = m20 * vector.x + m21 * vector.y + m22 * vector.z;

        return new Vector3f(x, y, z);
    }


    public Matrix4f transpose() {

        float temp;

        // Swap elements across the main diagonal
        temp = m01;
        m01 = m10;
        m10 = temp;

        temp = m02;
        m02 = m20;
        m20 = temp;

        temp = m03;
        m03 = m30;
        m30 = temp;

        temp = m12;
        m12 = m21;
        m21 = temp;

        temp = m13;
        m13 = m31;
        m31 = temp;

        temp = m23;
        m23 = m32;
        m32 = temp;

        return this;
    }

    public float determinant() {
        float f =
                m00
                        * ((m11 * m22 * m33 + m12 * m23 * m31 + m13 * m21 * m32)
                        - m13 * m22 * m31
                        - m11 * m23 * m32
                        - m12 * m21 * m33);
        f -= m01
                * ((m10 * m22 * m33 + m12 * m23 * m30 + m13 * m20 * m32)
                - m13 * m22 * m30
                - m10 * m23 * m32
                - m12 * m20 * m33);
        f += m02
                * ((m10 * m21 * m33 + m11 * m23 * m30 + m13 * m20 * m31)
                - m13 * m21 * m30
                - m10 * m23 * m31
                - m11 * m20 * m33);
        f -= m03
                * ((m10 * m21 * m32 + m11 * m22 * m30 + m12 * m20 * m31)
                - m12 * m21 * m30
                - m10 * m22 * m31
                - m11 * m20 * m32);
        return f;
    }

    public Vector3f getScale() {
        float scaleX = (float) Math.sqrt(m00 * m00 + m01 * m01 + m02 * m02);
        float scaleY = (float) Math.sqrt(m10 * m10 + m11 * m11 + m12 * m12);
        float scaleZ = (float) Math.sqrt(m20 * m20 + m21 * m21 + m22 * m22);

        return new Vector3f(scaleX, scaleY, scaleZ);
    }

    public Vector3f getTranslation() {
        return new Vector3f(m30, m31, m32);
    }

    public Quaternion getRotation() {
        float trace = m00 + m11 + m22;

        float qw, qx, qy, qz;

        if (trace > 0) {
            float s = 0.5f / (float) Math.sqrt(trace + 1.0);
            qw = 0.25f / s;
            qx = (m21 - m12) * s;
            qy = (m02 - m20) * s;
            qz = (m10 - m01) * s;
        } else if (m00 > m11 && m00 > m22) {
            float s = 2.0f * (float) Math.sqrt(1.0 + m00 - m11 - m22);
            qw = (m21 - m12) / s;
            qx = 0.25f * s;
            qy = (m01 + m10) / s;
            qz = (m02 + m20) / s;
        } else if (m11 > m22) {
            float s = 2.0f * (float) Math.sqrt(1.0 + m11 - m00 - m22);
            qw = (m02 - m20) / s;
            qx = (m01 + m10) / s;
            qy = 0.25f * s;
            qz = (m12 + m21) / s;
        } else {
            float s = 2.0f * (float) Math.sqrt(1.0 + m22 - m00 - m11);
            qw = (m10 - m01) / s;
            qx = (m02 + m20) / s;
            qy = (m12 + m21) / s;
            qz = 0.25f * s;
        }

        return new Quaternion(qx, qy, qz, qw);
    }

    public Matrix4f scale(Vector3f vec) {
        m00 *= vec.x;
        m11 *= vec.y;
        m22 *= vec.z;

        return this;
    }

    //create view matrix
    public Matrix4f lookAt(Vector3f eye, Vector3f center, Vector3f up) {
        Vector3f f = center.sub(eye).normalize(); // Calculate the forward vector
        Vector3f r = f.cross(up).normalize(); // Calculate the right vector
        Vector3f u = r.cross(f); // Calculate the up vector

        // Set the rotation part of the matrix
        m00 = r.x;
        m01 = r.y;
        m02 = r.z;
        m03 = 0.0f;
        m10 = u.x;
        m11 = u.y;
        m12 = u.z;
        m13 = 0.0f;
        m20 = -f.x;
        m21 = -f.y;
        m22 = -f.z;
        m23 = 0.0f;

        // Set the translation part of the matrix
        m30 = -r.dot(eye);
        m31 = -u.dot(eye);
        m32 = f.dot(eye);
        m33 = 1.0f;

        return this;
    }


    public Matrix4f createPerspectiveMatrix(float fov, float aspectRatio, float near, float far) {
        Matrix4f dest = new Matrix4f();
        float tanHalfFOV = (float) Math.tan(Math.toRadians(fov / 2));
        float range = near - far;

        dest.m00 = 1.0f / (aspectRatio * tanHalfFOV);
        dest.m01 = 0.0f;
        dest.m02 = 0.0f;
        dest.m03 = 0.0f;

        dest.m10 = 0.0f;
        dest.m11 = 1.0f / tanHalfFOV;
        dest.m12 = 0.0f;
        dest.m13 = 0.0f;

        dest.m20 = 0.0f;
        dest.m21 = 0.0f;
        dest.m22 = (-near - far) / range;
        dest.m23 = 2.0f * far * near / range;

        dest.m30 = 0.0f;
        dest.m31 = 0.0f;
        dest.m32 = 1.0f;
        dest.m33 = 0.0f;

        return dest;
    }


    public Matrix4f invert() {
        float det = determinant();

        if (Math.abs(det) <= EPSILON) {
            // The matrix is not invertible (determinant is too close to 0)
            // Handle this case as needed in your application, e.g., throw an exception
            throw new UnsupportedOperationException("Matrix is not invertible.");
        }

        float invDet = 1.0f / det;

        float t00 = (m11 * m22 * m33 + m12 * m23 * m31 + m13 * m21 * m32 - m13 * m22 * m31 - m11 * m23 * m32 - m12 * m21 * m33) * invDet;
        float t01 = (m01 * m22 * m33 + m02 * m23 * m31 + m03 * m21 * m32 - m03 * m22 * m31 - m01 * m23 * m32 - m02 * m21 * m33) * invDet;
        float t02 = (m01 * m12 * m33 + m02 * m13 * m31 + m03 * m11 * m32 - m03 * m12 * m31 - m01 * m13 * m32 - m02 * m11 * m33) * invDet;
        float t03 = (m01 * m12 * m23 + m02 * m13 * m21 + m03 * m11 * m22 - m03 * m12 * m21 - m01 * m13 * m22 - m02 * m11 * m23) * invDet;

        float t10 = (m10 * m22 * m33 + m12 * m23 * m30 + m13 * m20 * m32 - m13 * m22 * m30 - m10 * m23 * m32 - m12 * m20 * m33) * invDet;
        float t11 = (m00 * m22 * m33 + m02 * m23 * m30 + m03 * m20 * m32 - m03 * m22 * m30 - m00 * m23 * m32 - m02 * m20 * m33) * invDet;
        float t12 = (m00 * m12 * m33 + m02 * m13 * m30 + m03 * m10 * m32 - m03 * m12 * m30 - m00 * m13 * m32 - m02 * m10 * m33) * invDet;
        float t13 = (m00 * m12 * m23 + m02 * m13 * m20 + m03 * m10 * m22 - m03 * m12 * m20 - m00 * m13 * m22 - m02 * m10 * m23) * invDet;

        float t20 = (m10 * m21 * m33 + m11 * m23 * m30 + m13 * m20 * m31 - m13 * m21 * m30 - m10 * m23 * m31 - m11 * m20 * m33) * invDet;
        float t21 = (m00 * m21 * m33 + m01 * m23 * m30 + m03 * m20 * m31 - m03 * m21 * m30 - m00 * m23 * m31 - m01 * m20 * m33) * invDet;
        float t22 = (m00 * m11 * m33 + m01 * m13 * m30 + m03 * m10 * m31 - m03 * m11 * m30 - m00 * m13 * m31 - m01 * m10 * m33) * invDet;
        float t23 = (m00 * m11 * m23 + m01 * m13 * m20 + m03 * m10 * m21 - m03 * m11 * m20 - m00 * m13 * m21 - m01 * m10 * m23) * invDet;

        float t30 = (m10 * m21 * m32 + m11 * m22 * m30 + m12 * m20 * m31 - m12 * m21 * m30 - m10 * m22 * m31 - m11 * m20 * m32) * invDet;
        float t31 = (m00 * m21 * m32 + m01 * m22 * m30 + m02 * m20 * m31 - m02 * m21 * m30 - m00 * m22 * m31 - m01 * m20 * m32) * invDet;
        float t32 = (m00 * m11 * m32 + m01 * m12 * m30 + m02 * m10 * m31 - m02 * m11 * m30 - m00 * m12 * m31 - m01 * m10 * m32) * invDet;
        float t33 = (m00 * m11 * m22 + m01 * m12 * m20 + m02 * m10 * m21 - m02 * m11 * m20 - m00 * m12 * m21 - m01 * m10 * m22) * invDet;

        // Set the matrix values to the inverted values
        this.m00 = t00;
        this.m01 = t01;
        this.m02 = t02;
        this.m03 = t03;

        this.m10 = t10;
        this.m11 = t11;
        this.m12 = t12;
        this.m13 = t13;

        this.m20 = t20;
        this.m21 = t21;
        this.m22 = t22;
        this.m23 = t23;

        this.m30 = t30;
        this.m31 = t31;
        this.m32 = t32;
        this.m33 = t33;

        return this;

    }

    public Matrix4f negate() {
        m00 = -m00;
        m01 = -m01;
        m02 = -m02;
        m03 = -m03;
        m10 = -m10;
        m11 = -m11;
        m12 = -m12;
        m13 = -m13;
        m20 = -m20;
        m21 = -m21;
        m22 = -m22;
        m23 = -m23;
        m30 = -m30;
        m31 = -m31;
        m32 = -m32;
        m33 = -m33;

        return this;
    }

    public Matrix4f rotateGeneric(Quaternion quaternion) {
        // Normalize the quaternion if it's not already normalized
        Quaternion quaternionNorm = quaternion.normalize();

        float qx = quaternionNorm.x;
        float qy = quaternionNorm.y;
        float qz = quaternionNorm.z;
        float qw = quaternionNorm.w;

        float xx = qx * qx;
        float yy = qy * qy;
        float zz = qz * qz;
        float xy = qx * qy;
        float xz = qx * qz;
        float yz = qy * qz;
        float wx = qw * qx;
        float wy = qw * qy;
        float wz = qw * qz;

        float m00 = 1.0f - 2.0f * (yy + zz);
        float m01 = 2.0f * (xy - wz);
        float m02 = 2.0f * (xz + wy);
        float m03 = 0.0f;

        float m10 = 2.0f * (xy + wz);
        float m11 = 1.0f - 2.0f * (xx + zz);
        float m12 = 2.0f * (yz - wx);
        float m13 = 0.0f;

        float m20 = 2.0f * (xz - wy);
        float m21 = 2.0f * (yz + wx);
        float m22 = 1.0f - 2.0f * (xx + yy);
        float m23 = 0.0f;

        // Apply the rotation to the current matrix
        float tempM00 = this.m00 * m00 + this.m01 * m10 + this.m02 * m20;
        float tempM01 = this.m00 * m01 + this.m01 * m11 + this.m02 * m21;
        float tempM02 = this.m00 * m02 + this.m01 * m12 + this.m02 * m22;
        float tempM03 = this.m00 * m03 + this.m01 * m13 + this.m02 * m23 + this.m03;

        float tempM10 = this.m10 * m00 + this.m11 * m10 + this.m12 * m20;
        float tempM11 = this.m10 * m01 + this.m11 * m11 + this.m12 * m21;
        float tempM12 = this.m10 * m02 + this.m11 * m12 + this.m12 * m22;
        float tempM13 = this.m10 * m03 + this.m11 * m13 + this.m12 * m23 + this.m13;

        float tempM20 = this.m20 * m00 + this.m21 * m10 + this.m22 * m20;
        float tempM21 = this.m20 * m01 + this.m21 * m11 + this.m22 * m21;
        float tempM22 = this.m20 * m02 + this.m21 * m12 + this.m22 * m22;
        float tempM23 = this.m20 * m03 + this.m21 * m13 + this.m22 * m23 + this.m23;

        this.m00 = tempM00;
        this.m01 = tempM01;
        this.m02 = tempM02;
        this.m03 = tempM03;

        this.m10 = tempM10;
        this.m11 = tempM11;
        this.m12 = tempM12;
        this.m13 = tempM13;

        this.m20 = tempM20;
        this.m21 = tempM21;
        this.m22 = tempM22;
        this.m23 = tempM23;

        this.m30 = 0.0f;
        this.m31 = 0.0f;
        this.m32 = 0.0f;
        this.m33 = 1.0f;
        return this;
    }

    public static Vector3f localToWorldDirection(Vector3f local, Matrix4f transform) {
        // Apply only the rotational part of the transformation matrix to the local direction
        float x = local.x * transform.m00 + local.y * transform.m10 + local.z * transform.m20;
        float y = local.x * transform.m01 + local.y * transform.m11 + local.z * transform.m21;
        float z = local.x * transform.m02 + local.y * transform.m12 + local.z * transform.m22;

        return new Vector3f(x, y, z);
    }

    public static Vector3f worldToLocalDirection(Vector3f world, Matrix4f transform) {
        // Apply the inverse of the rotational part of the transformation matrix to the world direction
        float x = world.x * transform.m00 + world.y * transform.m01 + world.z * transform.m02;
        float y = world.x * transform.m10 + world.y * transform.m11 + world.z * transform.m12;
        float z = world.x * transform.m20 + world.y * transform.m21 + world.z * transform.m22;

        return new Vector3f(x, y, z);
    }

    public float getM00() {
        return m00;
    }

    public void setM00(float m00) {
        this.m00 = m00;
    }

    public float getM01() {
        return m01;
    }

    public void setM01(float m01) {
        this.m01 = m01;
    }

    public float getM02() {
        return m02;
    }

    public void setM02(float m02) {
        this.m02 = m02;
    }

    public float getM03() {
        return m03;
    }

    public void setM03(float m03) {
        this.m03 = m03;
    }

    public float getM10() {
        return m10;
    }

    public void setM10(float m10) {
        this.m10 = m10;
    }

    public float getM11() {
        return m11;
    }

    public void setM11(float m11) {
        this.m11 = m11;
    }

    public float getM12() {
        return m12;
    }

    public void setM12(float m12) {
        this.m12 = m12;
    }

    public float getM13() {
        return m13;
    }

    public void setM13(float m13) {
        this.m13 = m13;
    }

    public float getM20() {
        return m20;
    }

    public void setM20(float m20) {
        this.m20 = m20;
    }

    public float getM21() {
        return m21;
    }

    public void setM21(float m21) {
        this.m21 = m21;
    }

    public float getM22() {
        return m22;
    }

    public void setM22(float m22) {
        this.m22 = m22;
    }

    public float getM23() {
        return m23;
    }

    public void setM23(float m23) {
        this.m23 = m23;
    }

    public float getM30() {
        return m30;
    }

    public void setM30(float m30) {
        this.m30 = m30;
    }

    public float getM31() {
        return m31;
    }

    public void setM31(float m31) {
        this.m31 = m31;
    }

    public float getM32() {
        return m32;
    }

    public void setM32(float m32) {
        this.m32 = m32;
    }

    public float getM33() {
        return m33;
    }

    public void setM33(float m33) {
        this.m33 = m33;
    }

    @Override
    public String toString() {
        return "Matrix4f{" +
                "m00=" + m00 +
                ", m01=" + m01 +
                ", m02=" + m02 +
                ", m03=" + m03 +
                ", m10=" + m10 +
                ", m11=" + m11 +
                ", m12=" + m12 +
                ", m13=" + m13 +
                ", m20=" + m20 +
                ", m21=" + m21 +
                ", m22=" + m22 +
                ", m23=" + m23 +
                ", m30=" + m30 +
                ", m31=" + m31 +
                ", m32=" + m32 +
                ", m33=" + m33 +
                '}';
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Matrix4f matrix4f = (Matrix4f) o;
        return Float.compare(matrix4f.m00, m00) == 0 && Float.compare(matrix4f.m01, m01) == 0 &&
                Float.compare(matrix4f.m02, m02) == 0 && Float.compare(matrix4f.m03, m03) == 0 &&
                Float.compare(matrix4f.m10, m10) == 0 && Float.compare(matrix4f.m11, m11) == 0 &&
                Float.compare(matrix4f.m12, m12) == 0 && Float.compare(matrix4f.m13, m13) == 0 &&
                Float.compare(matrix4f.m20, m20) == 0 && Float.compare(matrix4f.m21, m21) == 0 &&
                Float.compare(matrix4f.m22, m22) == 0 && Float.compare(matrix4f.m23, m23) == 0 &&
                Float.compare(matrix4f.m30, m30) == 0 && Float.compare(matrix4f.m31, m31) == 0 &&
                Float.compare(matrix4f.m32, m32) == 0 && Float.compare(matrix4f.m33, m33) == 0;
    }

    @Override
    public int hashCode() {
        return Objects.hash(m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31, m32, m33);
    }

    public Vector3f transformInverse(Vector3f vector) {
        Vector3f tmp = new Vector3f(vector);
        tmp.x -= m03;
        tmp.y -= m13;
        tmp.z -= m23;
        return new Vector3f(
                tmp.x * m00 +
                        tmp.y * m10 +
                        tmp.z * m20,

                tmp.x * m01 +
                        tmp.y * m11 +
                        tmp.z * m21,

                tmp.x * m02 +
                        tmp.y * m12 +
                        tmp.z * m22
        );
    }

    public Vector3f transformDirection(Vector3f vector) {
        return new Vector3f(
                vector.x * m00 +
                        vector.y * m01 +
                        vector.z * m02,

                vector.x * m10 +
                        vector.y * m11 +
                        vector.z * m12,

                vector.x * m20 +
                        vector.y * m21 +
                        vector.z * m22
        );
    }
}
