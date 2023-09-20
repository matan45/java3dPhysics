package math;

import java.util.Objects;

public class Vector3f {
    public float x;
    public float y;
    public float z;

    public static final Vector3f Zero = new Vector3f();
    public static final Vector3f XAxis = new Vector3f(1, 0, 0);
    public static final Vector3f YAxis = new Vector3f(0, 1, 0);
    public static final Vector3f ZAxis = new Vector3f(0, 0, 1);

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
        this.x = other.x;
        this.y = other.y;
        this.z = other.z;
    }

    public static Vector3f min(Vector3f v1, Vector3f v2) {
        float minX = Math.min(v1.x, v2.x);
        float minY = Math.min(v1.y, v2.y);
        float minZ = Math.min(v1.z, v2.z);

        return new Vector3f(minX, minY, minZ);
    }

    public static Vector3f max(Vector3f v1, Vector3f v2) {
        float maxX = Math.max(v1.x, v2.x);
        float maxY = Math.max(v1.y, v2.y);
        float maxZ = Math.max(v1.z, v2.z);

        return new Vector3f(maxX, maxY, maxZ);
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

    public Vector3f mul(Vector3f other) {
        return new Vector3f(x * other.x, y * other.y, z * other.z);
    }

    public Vector3f cross(Vector3f other) {
        return new Vector3f(y * other.z - z * other.y, other.x * z - other.z * x,
                x * other.y - y * other.x);
    }

    public Vector3f normalize() {
        float l = length();
        if (l != 0)
            return new Vector3f(x / l, y / l, z / l);

        return new Vector3f(x, y, z);
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

    public float distance(Vector3f point) {
        float dx = x - point.x;
        float dy = y - point.y;
        float dz = z - point.z;
        return (float) Math.sqrt(dx * dx + dy * dy + dz * dz);
    }

    public Vector3f rotate(Quaternion rotation) {
        // Convert the Vector3f to a Quaternion4f for the rotation
        Quaternion vectorQuaternion = new Quaternion(x, y, z, 0);

        // Perform the rotation by multiplying the rotation quaternion
        Quaternion result = rotation.mul(vectorQuaternion).mul(rotation.conjugate());

        // Update the Vector3f with the rotated values
        return new Vector3f(result.x, result.y, result.z);
    }

    public void set(Vector3f point) {
        this.x = point.x;
        this.y = point.y;
        this.z = point.z;
    }

    public Vector3f negate() {
        return new Vector3f(x, y, z).mul(-1);
    }

    public boolean isBetween(Vector3f start, Vector3f end) {
        // Check if the current vector is between the start and end vectors in all dimensions.
        boolean betweenX = (start.x <= this.x && this.x <= end.x) || (end.x <= this.x && this.x <= start.x);
        boolean betweenY = (start.y <= this.y && this.y <= end.y) || (end.y <= this.y && this.y <= start.y);
        boolean betweenZ = (start.z <= this.z && this.z <= end.z) || (end.z <= this.z && this.z <= start.z);

        // Return true if the current vector is between the start and end vectors in all dimensions.
        return betweenX && betweenY && betweenZ;
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

    public Vector3f lerp(Vector3f end, float t) {
        // Interpolate each component (x, y, z) of the vector separately
        float lerpedX = this.x + (end.x - this.x) * t;
        float lerpedY = this.y + (end.y - this.y) * t;
        float lerpedZ = this.z + (end.z - this.z) * t;

        // Create a new Vector3f with the interpolated values
        return new Vector3f(lerpedX, lerpedY, lerpedZ);
    }

    public void clear() {
        this.x = 0;
        this.y = 0;
        this.z = 0;
    }
}
