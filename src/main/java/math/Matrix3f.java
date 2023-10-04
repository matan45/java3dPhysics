package math;

import java.util.Objects;

import static math.Const.EPSILON;

public class Matrix3f {
    float m00, m01, m02, m03;
    float m10, m11, m12, m13;
    float m20, m21, m22, m23;

    public Matrix3f() {
        setIdentity();
    }

    public Matrix3f(Matrix4f other) {
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
    }

    public Matrix3f setIdentity() {
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

        return this;
    }

    public void setMatrix(Matrix3f other) {
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
    }

    public Vector3f transform(Vector3f position) {
        float x = m00 * position.x + m01 * position.y + m02 * position.z + m03;
        float y = m10 * position.x + m11 * position.y + m12 * position.z + m13;
        float z = m20 * position.x + m21 * position.y + m22 * position.z + m23;

        return new Vector3f(x, y, z);
    }

    public Matrix3f add(Matrix3f right) {

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

        return this;
    }

    public Matrix3f sub(Matrix3f right) {

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

        return this;
    }

    public Matrix3f mul(Matrix3f right) {

        m00 = m00 * right.m00 + m10 * right.m01 + m20 * right.m02;
        m01 = m01 * right.m00 + m11 * right.m01 + m21 * right.m02;
        m02 = m02 * right.m00 + m12 * right.m01 + m22 * right.m02;
        m03 = m03 * right.m00 + m13 * right.m01 + m23 * right.m02;
        m10 = m00 * right.m10 + m10 * right.m11 + m20 * right.m12;
        m11 = m01 * right.m10 + m11 * right.m11 + m21 * right.m12;
        m12 = m02 * right.m10 + m12 * right.m11 + m22 * right.m12;
        m13 = m03 * right.m10 + m13 * right.m11 + m23 * right.m12;
        m20 = m00 * right.m20 + m10 * right.m21 + m20 * right.m22;
        m21 = m01 * right.m20 + m11 * right.m21 + m21 * right.m22;
        m22 = m02 * right.m20 + m12 * right.m21 + m22 * right.m22;
        m23 = m03 * right.m20 + m13 * right.m21 + m23 * right.m22;

        return this;
    }

    public Matrix3f scale(Vector3f scale) {
        m00 = m00 * scale.x;
        m01 = m01 * scale.x;
        m02 = m02 * scale.x;
        m10 = m10 * scale.y;
        m11 = m11 * scale.y;
        m12 = m12 * scale.y;
        m20 = m20 * scale.z;
        m21 = m21 * scale.z;
        m22 = m22 * scale.z;
        return this;
    }

    public Matrix3f rotation(Quaternion quaternion) {
        float w2 = quaternion.w * quaternion.w;
        float x2 = quaternion.x * quaternion.x;
        float y2 = quaternion.y * quaternion.y;
        float z2 = quaternion.z * quaternion.z;
        float zw = quaternion.z * quaternion.w, dzw = zw + zw;
        float xy = quaternion.x * quaternion.y, dxy = xy + xy;
        float xz = quaternion.x * quaternion.z, dxz = xz + xz;
        float yw = quaternion.y * quaternion.w, dyw = yw + yw;
        float yz = quaternion.y * quaternion.z, dyz = yz + yz;
        float xw = quaternion.x * quaternion.w, dxw = xw + xw;
        m00 = w2 + x2 - z2 - y2;
        m01 = dxy + dzw;
        m02 = dxz - dyw;
        m10 = -dzw + dxy;
        m11 = y2 - z2 + w2 - x2;
        m12 = dyz + dxw;
        m20 = dyw + dxz;
        m21 = dyz - dxw;
        m22 = z2 - y2 - x2 + w2;

        return this;
    }


    public Matrix3f transpose() {

        float m00 = this.m00;
        float m01 = this.m10;
        float m02 = this.m20;
        float m10 = this.m01;
        float m11 = this.m11;
        float m12 = this.m21;
        float m20 = this.m02;
        float m21 = this.m12;
        float m22 = this.m22;

        this.m00 = m00;
        this.m01 = m01;
        this.m02 = m02;
        this.m10 = m10;
        this.m11 = m11;
        this.m12 = m12;
        this.m20 = m20;
        this.m21 = m21;
        this.m22 = m22;

        return this;
    }

    public float determinant() {
        return m00 * (m11 * m22 - m12 * m21)
                + m01 * (m12 * m20 - m10 * m22)
                + m02 * (m10 * m21 - m11 * m20);
    }

    public Matrix3f invert() {
        float det = determinant();

        if (Math.abs(det) <= EPSILON) {
            // The matrix is not invertible (determinant is too close to 0)
            // Handle this case as needed in your application, e.g., throw an exception
            throw new UnsupportedOperationException("Matrix is not invertible.");
        }

        float invDet = 1.0f / det;

        float t00 = (m11 * m22 + m12 * m23 + m13 * m21 - m13 * m22 - m11 * m23 - m12 * m21) * invDet;
        float t01 = (m01 * m22 + m02 * m23 + m03 * m21 - m03 * m22 - m01 * m23 - m02 * m21) * invDet;
        float t02 = (m01 * m12 + m02 * m13 + m03 * m11 - m03 * m12 - m01 * m13 - m02 * m11) * invDet;
        float t03 = (m01 * m12 * m23 + m02 * m13 * m21 + m03 * m11 * m22 - m03 * m12 * m21 - m01 * m13 * m22 - m02 * m11 * m23) * invDet;

        float t10 = (m10 * m22 + m12 * m23 + m13 * m20 - m13 * m22 - m10 * m23 - m12 * m20) * invDet;
        float t11 = (m00 * m22 + m02 * m23 + m03 * m20 - m03 * m22 - m00 * m23 - m02 * m20) * invDet;
        float t12 = (m00 * m12 + m02 * m13 + m03 * m10 - m03 * m12 - m00 * m13 - m02 * m10) * invDet;
        float t13 = (m00 * m12 * m23 + m02 * m13 * m20 + m03 * m10 * m22 - m03 * m12 * m20 - m00 * m13 * m22 - m02 * m10 * m23) * invDet;

        float t20 = (m10 * m21 + m11 * m23 + m13 * m20 - m13 * m21 - m10 * m23 - m11 * m20) * invDet;
        float t21 = (m00 * m21 + m01 * m23 + m03 * m20 - m03 * m21 - m00 * m23 - m01 * m20) * invDet;
        float t22 = (m00 * m11 + m01 * m13 + m03 * m10 - m03 * m11 - m00 * m13 - m01 * m10) * invDet;
        float t23 = (m00 * m11 * m23 + m01 * m13 * m20 + m03 * m10 * m21 - m03 * m11 * m20 - m00 * m13 * m21 - m01 * m10 * m23) * invDet;


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

        return this;
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

    @Override
    public String toString() {
        return "Matrix3f{" +
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
                '}';
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Matrix3f matrix3f = (Matrix3f) o;
        return Float.compare(m00, matrix3f.m00) == 0 && Float.compare(m01, matrix3f.m01) == 0 && Float.compare(m02, matrix3f.m02) == 0 && Float.compare(m03, matrix3f.m03) == 0 && Float.compare(m10, matrix3f.m10) == 0 && Float.compare(m11, matrix3f.m11) == 0 && Float.compare(m12, matrix3f.m12) == 0 && Float.compare(m13, matrix3f.m13) == 0 && Float.compare(m20, matrix3f.m20) == 0 && Float.compare(m21, matrix3f.m21) == 0 && Float.compare(m22, matrix3f.m22) == 0 && Float.compare(m23, matrix3f.m23) == 0;
    }

    @Override
    public int hashCode() {
        return Objects.hash(m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23);
    }
}
