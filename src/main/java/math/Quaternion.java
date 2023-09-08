package math;

import java.util.Objects;

public class Quaternion {
    public float x;
    public float y;
    public float z;
    public float w;

    public Quaternion() {
        this.w = 1.0f;
    }

    public Quaternion(float x, float y, float z, float w) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.w = w;
    }

    public Quaternion(Quaternion other) {
        this.x = other.x;
        this.y = other.y;
        this.z = other.z;
        this.w = other.w;
    }

    public float getX() {
        return x;
    }

    public void setX(float x) {
        this.x = x;
    }

    public float getY() {
        return y;
    }

    public void setY(float y) {
        this.y = y;
    }

    public float getZ() {
        return z;
    }

    public void setZ(float z) {
        this.z = z;
    }

    public float getW() {
        return w;
    }

    public void setW(float w) {
        this.w = w;
    }

    public float lengthSquared() {
        return x * x + y * y + z * z + w * w;
    }

    public Quaternion normalize() {
        float invNorm = 1 / lengthSquared();

        return new Quaternion(x * invNorm, y * invNorm, z * invNorm, w * invNorm);
    }

    public Quaternion add(Quaternion other) {
        return new Quaternion(this.x + other.x, this.y + other.y, this.z + other.z, this.w + other.w);
    }

    public Quaternion sub(Quaternion other) {
        return new Quaternion(this.x - other.x, this.y - other.y, this.z - other.z, this.w - other.w);
    }

    public float dot(Quaternion other) {
        return this.x * other.x + this.y * other.y + this.z * other.z + this.w * other.w;
    }

    public Quaternion negate() {
        return new Quaternion(-x, -y, -z, -w);
    }

    public Quaternion scale(float scale) {
        return new Quaternion(x * scale, y * scale, w * scale, z * scale);
    }

    public Quaternion mul(Quaternion right) {
        return new Quaternion(x * right.w + w * right.x + y * right.z
                - z * right.y, y * right.w + w * right.y
                + z * right.x - x * right.z, z * right.w
                + w * right.z + x * right.y - y * right.x,
                w * right.w - x * right.x - y * right.y
                        - z * right.z);
    }

    public Quaternion mulInverse(Quaternion right) {
        float n = right.lengthSquared();
        // zero-div may occur.
        n = (n == 0.0 ? n : 1 / n);
        // store on stack once for aliasing-safety

        return new Quaternion
                ((x * right.w - w * right.x - y
                        * right.z + z * right.y)
                        * n, (y * right.w - w * right.y - z
                        * right.x + x * right.z)
                        * n, (z * right.w - w * right.z - x
                        * right.y + y * right.x)
                        * n, (w * right.w + x * right.x + y
                        * right.y + z * right.z)
                        * n);
    }

    public void set(Quaternion q) {
        this.x = q.x;
        this.y = q.y;
        this.z = q.z;
        this.w = q.w;
    }

    public Quaternion setIdentity() {
        return new Quaternion(0, 0, 0, 1);
    }

    public Quaternion rotationAxis(float angle, Vector3f axis) {
        float xAxis = axis.x;
        float yAxis = axis.y;
        float zAxis = axis.z;
        float n = (float) Math.sqrt(x * x + y * y + z * z);
        // zero-div may occur.
        float s = (float) (Math.sin(0.5 * angle) / n);
        xAxis *= s;
        yAxis *= s;
        zAxis *= s;
        float wAngle = (float) Math.cos(0.5 * angle);
        return new Quaternion(xAxis, yAxis, zAxis, wAngle);
    }

    public static Quaternion setFromMatrix(Matrix4f m, Quaternion q) {
        return q.setFromMat(m.m00, m.m01, m.m02, m.m10, m.m11, m.m12, m.m20,
                m.m21, m.m22);
    }

    private Quaternion setFromMat(float m00, float m01, float m02, float m10,
                                  float m11, float m12, float m20, float m21, float m22) {

        float s;
        float tr = m00 + m11 + m22;
        if (tr >= 0.0) {
            s = (float) Math.sqrt(tr + 1.0);
            w = s * 0.5f;
            s = 0.5f / s;
            x = (m21 - m12) * s;
            y = (m02 - m20) * s;
            z = (m10 - m01) * s;
        } else {
            float max = Math.max(Math.max(m00, m11), m22);
            if (max == m00) {
                s = (float) Math.sqrt(m00 - (m11 + m22) + 1.0);
                x = s * 0.5f;
                s = 0.5f / s;
                y = (m01 + m10) * s;
                z = (m20 + m02) * s;
                w = (m21 - m12) * s;
            } else if (max == m11) {
                s = (float) Math.sqrt(m11 - (m22 + m00) + 1.0);
                y = s * 0.5f;
                s = 0.5f / s;
                z = (m12 + m21) * s;
                x = (m01 + m10) * s;
                w = (m02 - m20) * s;
            } else {
                s = (float) Math.sqrt(m22 - (m00 + m11) + 1.0);
                z = s * 0.5f;
                s = 0.5f / s;
                x = (m20 + m02) * s;
                y = (m12 + m21) * s;
                w = (m10 - m01) * s;
            }
        }
        return this;
    }

    @Override
    public String toString() {
        return "Quaternion{" +
                "x=" + x +
                ", y=" + y +
                ", z=" + z +
                ", w=" + w +
                '}';
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Quaternion that = (Quaternion) o;
        return Float.compare(that.x, x) == 0 && Float.compare(that.y, y) == 0
                && Float.compare(that.z, z) == 0 && Float.compare(that.w, w) == 0;
    }

    @Override
    public int hashCode() {
        return Objects.hash(x, y, z, w);
    }
}
