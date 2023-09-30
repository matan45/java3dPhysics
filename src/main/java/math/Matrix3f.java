package math;

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
}
