package math;

import java.util.Objects;

public class Vector2f {
    public float x;
    public float y;

    public static final Vector2f Zero = new Vector2f();
    public static final Vector2f XAxis = new Vector2f(1, 0);
    public static final Vector2f YAxis = new Vector2f(0, 1);

    public Vector2f(float x, float y) {
        this.x = x;
        this.y = y;
    }

    public Vector2f() {
        this.x = 0;
        this.y = 0;
    }

    public Vector2f(Vector2f other) {
        this.x = other.x;
        this.y = other.y;
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

    public static Vector2f min(Vector2f v1, Vector2f v2) {
        float minX = Math.min(v1.x, v2.x);
        float minY = Math.min(v1.y, v2.y);

        return new Vector2f(minX, minY);
    }

    public static Vector2f max(Vector2f v1, Vector2f v2) {
        float maxX = Math.max(v1.x, v2.x);
        float maxY = Math.max(v1.y, v2.y);

        return new Vector2f(maxX, maxY);
    }

    public float dot(Vector2f other) {
        return x * other.x + y * other.y;
    }

    public float distanceSquared(Vector2f other) {
        float dx = x - other.x;
        float dy = y - other.y;
        return dx * dx + dy * dy;
    }

    public Vector2f add(Vector2f other) {
        return new Vector2f(x + other.x, y + other.y);
    }

    public Vector2f mul(float s) {
        return new Vector2f(x * s, y * s);
    }

    public static Vector2f project(Vector2f length, Vector2f direction) {
        float dot = length.dot(direction);
        float magSq = direction.lengthSquared();
        return direction.mul(dot / magSq);
    }

    public Vector2f normalize() {
        float l = length();
        if (l != 0)
            return new Vector2f(x / l, y / l);

        return new Vector2f(x, y);
    }

    public Vector2f sub(Vector2f other) {
        return new Vector2f(x - other.x, y - other.y);
    }

    public float length() {
        return (float) Math.sqrt(lengthSquared());
    }

    public float lengthSquared() {
        return x * x + y * y;
    }

    public Vector2f div(float v) {
        return new Vector2f(x / v, y / v);
    }

    public float distance(Vector2f point) {
        float dx = x - point.x;
        float dy = y - point.y;
        return (float) Math.sqrt(dx * dx + dy * dy);
    }

    public void set(Vector2f point) {
        this.x = point.x;
        this.y = point.y;
    }

    public Vector2f negate() {
        return new Vector2f(x, y).mul(-1);
    }

    public boolean isBetween(Vector2f start, Vector2f end) {
        // Check if the current vector is between the start and end vectors in all dimensions.
        boolean betweenX = (start.x <= this.x && this.x <= end.x) || (end.x <= this.x && this.x <= start.x);
        boolean betweenY = (start.y <= this.y && this.y <= end.y) || (end.y <= this.y && this.y <= start.y);

        // Return true if the current vector is between the start and end vectors in all dimensions.
        return betweenX && betweenY;
    }


    @Override
    public String toString() {
        return "Vector2f{" +
                "x=" + x +
                ", y=" + y +
                '}';
    }


    @Override
    public int hashCode() {
        return Objects.hash(x, y);
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Vector2f vector2f = (Vector2f) o;
        return Float.compare(vector2f.x, x) == 0 && Float.compare(vector2f.y, y) == 0;
    }
}
