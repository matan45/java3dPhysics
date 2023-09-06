package math;

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

    public Quaternion add(float x, float y, float z, float w) {
        return new Quaternion(this.x + x, this.y + y, this.z + z, this.w + w);
    }

    public Quaternion sub(float x, float y, float z, float w) {
        return new Quaternion(this.x - x, this.y - y, this.z - z, this.w - w);
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

    public static Quaternion mul(Quaternion left, Quaternion right) {
        return new Quaternion(left.x * right.w + left.w * right.x + left.y * right.z
                - left.z * right.y, left.y * right.w + left.w * right.y
                + left.z * right.x - left.x * right.z, left.z * right.w
                + left.w * right.z + left.x * right.y - left.y * right.x,
                left.w * right.w - left.x * right.x - left.y * right.y
                        - left.z * right.z);
    }

    public static Quaternion mulInverse(Quaternion left, Quaternion right) {
        float n = right.lengthSquared();
        // zero-div may occur.
        n = (n == 0.0 ? n : 1 / n);
        // store on stack once for aliasing-safty

        return new Quaternion
                ((left.x * right.w - left.w * right.x - left.y
                        * right.z + left.z * right.y)
                        * n, (left.y * right.w - left.w * right.y - left.z
                        * right.x + left.x * right.z)
                        * n, (left.z * right.w - left.w * right.z - left.x
                        * right.y + left.y * right.x)
                        * n, (left.w * right.w + left.x * right.x + left.y
                        * right.y + left.z * right.z)
                        * n);
    }

    public void setFromAxisAngle(Vector4f a1) {
        x = a1.x;
        y = a1.y;
        z = a1.z;
        float n = (float) Math.sqrt(x * x + y * y + z * z);
        // zero-div may occur.
        float s = (float) (Math.sin(0.5 * a1.w) / n);
        x *= s;
        y *= s;
        z *= s;
        w = (float) Math.cos(0.5 * a1.w);
    }

    public Quaternion setFromMatrix(Matrix4f m) {
        return setFromMat(m.m00, m.m01, m.m02, m.m10, m.m11, m.m12, m.m20,
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
}
