package math;

import java.util.Objects;

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
        Matrix4f dest = new Matrix4f();

        dest.m00 = m00 + right.m00;
        dest.m01 = m01 + right.m01;
        dest.m02 = m02 + right.m02;
        dest.m03 = m03 + right.m03;
        dest.m10 = m10 + right.m10;
        dest.m11 = m11 + right.m11;
        dest.m12 = m12 + right.m12;
        dest.m13 = m13 + right.m13;
        dest.m20 = m20 + right.m20;
        dest.m21 = m21 + right.m21;
        dest.m22 = m22 + right.m22;
        dest.m23 = m23 + right.m23;
        dest.m30 = m30 + right.m30;
        dest.m31 = m31 + right.m31;
        dest.m32 = m32 + right.m32;
        dest.m33 = m33 + right.m33;

        return dest;
    }

    public Matrix4f sub(Matrix4f right) {
        Matrix4f dest = new Matrix4f();

        dest.m00 = m00 - right.m00;
        dest.m01 = m01 - right.m01;
        dest.m02 = m02 - right.m02;
        dest.m03 = m03 - right.m03;
        dest.m10 = m10 - right.m10;
        dest.m11 = m11 - right.m11;
        dest.m12 = m12 - right.m12;
        dest.m13 = m13 - right.m13;
        dest.m20 = m20 - right.m20;
        dest.m21 = m21 - right.m21;
        dest.m22 = m22 - right.m22;
        dest.m23 = m23 - right.m23;
        dest.m30 = m30 - right.m30;
        dest.m31 = m31 - right.m31;
        dest.m32 = m32 - right.m32;
        dest.m33 = m33 - right.m33;

        return dest;
    }

    public Matrix4f mul(Matrix4f right) {
        Matrix4f dest = new Matrix4f();

        dest.m00 = m00 * right.m00 + m10 * right.m01 + m20 * right.m02 + m30 * right.m03;
        dest.m01 = m01 * right.m00 + m11 * right.m01 + m21 * right.m02 + m31 * right.m03;
        dest.m02 = m02 * right.m00 + m12 * right.m01 + m22 * right.m02 + m32 * right.m03;
        dest.m03 = m03 * right.m00 + m13 * right.m01 + m23 * right.m02 + m33 * right.m03;
        dest.m10 = m00 * right.m10 + m10 * right.m11 + m20 * right.m12 + m30 * right.m13;
        dest.m11 = m01 * right.m10 + m11 * right.m11 + m21 * right.m12 + m31 * right.m13;
        dest.m12 = m02 * right.m10 + m12 * right.m11 + m22 * right.m12 + m32 * right.m13;
        dest.m13 = m03 * right.m10 + m13 * right.m11 + m23 * right.m12 + m33 * right.m13;
        dest.m20 = m00 * right.m20 + m10 * right.m21 + m20 * right.m22 + m30 * right.m23;
        dest.m21 = m01 * right.m20 + m11 * right.m21 + m21 * right.m22 + m31 * right.m23;
        dest.m22 = m02 * right.m20 + m12 * right.m21 + m22 * right.m22 + m32 * right.m23;
        dest.m23 = m03 * right.m20 + m13 * right.m21 + m23 * right.m22 + m33 * right.m23;
        dest.m30 = m00 * right.m30 + m10 * right.m31 + m20 * right.m32 + m30 * right.m33;
        dest.m31 = m01 * right.m30 + m11 * right.m31 + m21 * right.m32 + m31 * right.m33;
        dest.m32 = m02 * right.m30 + m12 * right.m31 + m22 * right.m32 + m32 * right.m33;
        dest.m33 = m03 * right.m30 + m13 * right.m31 + m23 * right.m32 + m33 * right.m33;

        return dest;
    }

    public Matrix4f translate(Vector3f vec) {
        Matrix4f dest = new Matrix4f();

        dest.m30 += m00 * vec.x + m10 * vec.y + m20 * vec.z;
        dest.m31 += m01 * vec.x + m11 * vec.y + m21 * vec.z;
        dest.m32 += m02 * vec.x + m12 * vec.y + m22 * vec.z;
        dest.m33 += m03 * vec.x + m13 * vec.y + m23 * vec.z;

        return dest;
    }

    public Matrix4f transpose() {
        Matrix4f dest = new Matrix4f();
        dest.m00 = m00;
        dest.m01 = m01;
        dest.m02 = m20;
        dest.m03 = m30;
        dest.m10 = m10;
        dest.m11 = m11;
        dest.m12 = m21;
        dest.m13 = m31;
        dest.m20 = m02;
        dest.m21 = m12;
        dest.m22 = m22;
        dest.m23 = m32;
        dest.m30 = m03;
        dest.m31 = m13;
        dest.m32 = m23;
        dest.m33 = m33;

        return dest;
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

    public Matrix4f scale(Vector3f vec) {
        Matrix4f dest = new Matrix4f();

        dest.m00 = m00 * vec.x;
        dest.m01 = m01 * vec.x;
        dest.m02 = m02 * vec.x;
        dest.m03 = m03 * vec.x;
        dest.m10 = m10 * vec.y;
        dest.m11 = m11 * vec.y;
        dest.m12 = m12 * vec.y;
        dest.m13 = m13 * vec.y;
        dest.m20 = m20 * vec.z;
        dest.m21 = m21 * vec.z;
        dest.m22 = m22 * vec.z;
        dest.m23 = m23 * vec.z;
        return dest;
    }

    //create view matrix
    public Matrix4f lookAt(Vector3f eye, Vector3f center, Vector3f up) {
        Matrix4f dest = new Matrix4f();
        Vector3f f = center.sub(eye).normalize(); // Calculate the forward vector
        Vector3f r = f.cross(up).normalize(); // Calculate the right vector
        Vector3f u = r.cross(f); // Calculate the up vector

        // Set the rotation part of the matrix
        dest.m00 = r.x;
        dest.m01 = r.y;
        dest.m02 = r.z;
        dest.m03 = 0.0f;
        dest.m10 = u.x;
        dest.m11 = u.y;
        dest.m12 = u.z;
        dest.m13 = 0.0f;
        dest.m20 = -f.x;
        dest.m21 = -f.y;
        dest.m22 = -f.z;
        dest.m23 = 0.0f;

        // Set the translation part of the matrix
        dest.m30 = -r.dot(eye);
        dest.m31 = -u.dot(eye);
        dest.m32 = f.dot(eye);
        dest.m33 = 1.0f;

        return dest;
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
        float determinant = determinant();
        Matrix4f dest = new Matrix4f();

        if (determinant != 0) {
            /*
             * m00 m01 m02 m03
             * m10 m11 m12 m13
             * m20 m21 m22 m23
             * m30 m31 m32 m33
             */
            float determinant_inv = 1f / determinant;

            // first row
            float t00 = determinant3x3(m11, m12, m13, m21, m22, m23, m31, m32, m33);
            float t01 = -determinant3x3(m10, m12, m13, m20, m22, m23, m30, m32, m33);
            float t02 = determinant3x3(m10, m11, m13, m20, m21, m23, m30, m31, m33);
            float t03 = -determinant3x3(m10, m11, m12, m20, m21, m22, m30, m31, m32);
            // second row
            float t10 = -determinant3x3(m01, m02, m03, m21, m22, m23, m31, m32, m33);
            float t11 = determinant3x3(m00, m02, m03, m20, m22, m23, m30, m32, m33);
            float t12 = -determinant3x3(m00, m01, m03, m20, m21, m23, m30, m31, m33);
            float t13 = determinant3x3(m00, m01, m02, m20, m21, m22, m30, m31, m32);
            // third row
            float t20 = determinant3x3(m01, m02, m03, m11, m12, m13, m31, m32, m33);
            float t21 = -determinant3x3(m00, m02, m03, m10, m12, m13, m30, m32, m33);
            float t22 = determinant3x3(m00, m01, m03, m10, m11, m13, m30, m31, m33);
            float t23 = -determinant3x3(m00, m01, m02, m10, m11, m12, m30, m31, m32);
            // fourth row
            float t30 = -determinant3x3(m01, m02, m03, m11, m12, m13, m21, m22, m23);
            float t31 = determinant3x3(m00, m02, m03, m10, m12, m13, m20, m22, m23);
            float t32 = -determinant3x3(m00, m01, m03, m10, m11, m13, m20, m21, m23);
            float t33 = determinant3x3(m00, m01, m02, m10, m11, m12, m20, m21, m22);

            // transpose and divide by the determinant
            dest.m00 = t00 * determinant_inv;
            dest.m11 = t11 * determinant_inv;
            dest.m22 = t22 * determinant_inv;
            dest.m33 = t33 * determinant_inv;
            dest.m01 = t10 * determinant_inv;
            dest.m10 = t01 * determinant_inv;
            dest.m20 = t02 * determinant_inv;
            dest.m02 = t20 * determinant_inv;
            dest.m12 = t21 * determinant_inv;
            dest.m21 = t12 * determinant_inv;
            dest.m03 = t30 * determinant_inv;
            dest.m30 = t03 * determinant_inv;
            dest.m13 = t31 * determinant_inv;
            dest.m31 = t13 * determinant_inv;
            dest.m32 = t23 * determinant_inv;
            dest.m23 = t32 * determinant_inv;
            return dest;
        }
        return dest;
    }

    public Matrix4f negate() {
        Matrix4f dest = new Matrix4f();

        dest.m00 = -m00;
        dest.m01 = -m01;
        dest.m02 = -m02;
        dest.m03 = -m03;
        dest.m10 = -m10;
        dest.m11 = -m11;
        dest.m12 = -m12;
        dest.m13 = -m13;
        dest.m20 = -m20;
        dest.m21 = -m21;
        dest.m22 = -m22;
        dest.m23 = -m23;
        dest.m30 = -m30;
        dest.m31 = -m31;
        dest.m32 = -m32;
        dest.m33 = -m33;

        return dest;
    }

    public Matrix4f rotateGeneric(Quaternion quaternion) {
        Matrix4f dest = new Matrix4f();
        float w2 = quaternion.w * quaternion.w;
        float x2 = quaternion.x * quaternion.x;
        float y2 = quaternion.y * quaternion.y;
        float z2 = quaternion.z * quaternion.z;
        float zw = quaternion.z * quaternion.w;
        float xy = quaternion.x * quaternion.y;
        float xz = quaternion.x * quaternion.z;
        float yw = quaternion.y * quaternion.w;
        float yz = quaternion.y * quaternion.z;
        float xw = quaternion.x * quaternion.w;
        float rm00 = w2 + x2 - z2 - y2;
        float rm01 = xy + zw + zw + xy;
        float rm02 = xz - yw + xz - yw;
        float rm10 = -zw + xy - zw + xy;
        float rm11 = y2 - z2 + w2 - x2;
        float rm12 = yz + yz + xw + xw;
        float rm20 = yw + xz + xz + yw;
        float rm21 = yz + yz - xw - xw;
        float rm22 = z2 - y2 - x2 + w2;
        float nm00 = m00 * rm00 + m10 * rm01 + m20 * rm02;
        float nm01 = m01 * rm00 + m11 * rm01 + m21 * rm02;
        float nm02 = m02 * rm00 + m12 * rm01 + m22 * rm02;
        float nm03 = m03 * rm00 + m13 * rm01 + m23 * rm02;
        float nm10 = m00 * rm10 + m10 * rm11 + m20 * rm12;
        float nm11 = m01 * rm10 + m11 * rm11 + m21 * rm12;
        float nm12 = m02 * rm10 + m12 * rm11 + m22 * rm12;
        float nm13 = m03 * rm10 + m13 * rm11 + m23 * rm12;
        dest.m20 = m00 * rm20 + m10 * rm21 + m20 * rm22;
        dest.m21 = m01 * rm20 + m11 * rm21 + m21 * rm22;
        dest.m22 = m02 * rm20 + m12 * rm21 + m22 * rm22;
        dest.m23 = m03 * rm20 + m13 * rm21 + m23 * rm22;
        dest.m00 = nm00;
        dest.m01 = nm01;
        dest.m02 = nm02;
        dest.m03 = nm03;
        dest.m10 = nm10;
        dest.m11 = nm11;
        dest.m12 = nm12;
        dest.m13 = nm13;
        dest.m30 = m30;
        dest.m31 = m31;
        dest.m32 = m32;
        dest.m33 = m33;
        return dest;
    }

    private static float determinant3x3(float t00, float t01, float t02,
                                        float t10, float t11, float t12,
                                        float t20, float t21, float t22) {
        return t00 * (t11 * t22 - t12 * t21)
                + t01 * (t12 * t20 - t10 * t22)
                + t02 * (t10 * t21 - t11 * t20);
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
}
