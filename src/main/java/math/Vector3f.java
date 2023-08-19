package math;

import java.util.Objects;

public class Vector3f {
    public float x;
    public float y;
    public float z;

    public Vector3f(float x, float y, float z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public Vector3f() {
        this.x = 0;
        this.y = 0;
        this.z = 0;
    }

    public Vector3f(Vector3f other) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public float get(int index) {
        if (index == 0)
            return x;
        if (index == 1)
            return y;

        return z;
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

    public static Vector3f project(Vector3f length, Vector3f direction) {
        float dot = length.dot(direction);
        float magSq = direction.lengthSquared();
        return direction.mul(dot / magSq);
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Vector3f vector3f = (Vector3f) o;
        return Float.compare(vector3f.x, x) == 0 && Float.compare(vector3f.y, y) == 0 && Float.compare(vector3f.z, z) == 0;
    }

    @Override
    public int hashCode() {
        return Objects.hash(x, y, z);
    }

    @Override
    public String toString() {
        return "Vector3f{" +
                "x=" + x +
                ", y=" + y +
                ", z=" + z +
                '}';
    }

    public float dot(Vector3f other) {
        return x * other.x + y * other.y + z * other.z;
    }

    public float distanceSquared(Vector3f other) {
        float dx = x - other.x;
        float dy = y - other.y;
        float dz = z - other.z;
        return dx * dx + dy * dy + dz * dz;
    }

    public Vector3f add(Vector3f other) {
        return new Vector3f(x + other.x, y + other.y, z + other.z);
    }

    public Vector3f mul(float s) {
        return new Vector3f(x * s, y * s, z * s);
    }

    public Vector3f cross(Vector3f other) {
        return new Vector3f(y * other.z - z * other.y, other.x * z - other.z * x,
                x * other.y - y * other.x);
    }

    public Vector3f normalize() {
        float l = length();
        return new Vector3f(x / l, y / l, z / l);
    }

    public Vector3f sub(Vector3f other) {
        return new Vector3f(x - other.x, y - other.y, z - other.z);
    }

    public float length() {
        return (float) Math.sqrt(lengthSquared());
    }

    public float lengthSquared() {
        return x * x + y * y + z * z;
    }

    public Vector3f div(float v) {
        return new Vector3f(x / v, y / v, z / v);
    }
}