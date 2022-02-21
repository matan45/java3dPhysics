package math;

public class Vector2f {

    // Component x
    float x;

    // Component y
    float y;

    // Constructor
    public Vector2f() {
        zero();
    }

    // Constructor with arguments
    public Vector2f(float x, float y) {
        set(x, y);
    }

    // Copy-constructor
    public Vector2f(Vector2f vector) {
        set(vector);
    }

    // Return true if the vector is unit and false otherwise
    public boolean isUnit() {
        return Mathematics.ApproxEqual(lengthSquare(), 1.0f, Mathematics.MACHINE_EPSILON);
    }

    // Return true if the vector is the zero vector
    public boolean isZero() {
        return Mathematics.ApproxEqual(lengthSquare(), 0.0f, Mathematics.MACHINE_EPSILON);
    }

    // Scalar product of two vectors (public)
    public float dot(Vector2f vector) {
        return x * vector.x + y * vector.y;
    }

    // Overloaded operator for value access
    public float get(int index) {
        if (index == 0) {
            return x;
        } else if (index == 1) {
            return y;
        }
        throw new IllegalArgumentException("Unknown index: " + index);
    }

    public float getX() {
        return x;
    }

    public float getY() {
        return y;
    }

    // Return the length of the vector
    public float length() {
        return Mathematics.Sqrt(x * x + y * y);
    }

    // Return the square of the length of the vector
    public float lengthSquare() {
        return x * x + y * y;
    }

    // Return the axis with the maximal value
    public int getMaxAxis() {
        return (x < y ? 1 : 0);
    }

    // Return the axis with the minimal value
    public int getMinAxis() {
        return (x < y ? 0 : 1);
    }

    // Return the corresponding absolute value vector
    public Vector2f abs() {
        x = Math.abs(x);
        y = Math.abs(y);
        return this;
    }

    // Overloaded operator for addition with assignment
    public Vector2f add(Vector2f vector) {
        x += vector.x;
        y += vector.y;
        return this;
    }

    // Overloaded operator for division by a number with assignment
    public Vector2f divide(float number) {
        assert (number > Mathematics.MACHINE_EPSILON);
        x /= number;
        y /= number;
        return this;
    }

    // Overloaded operator for the negative of a vector
    public Vector2f invert() {
        x = -x;
        y = -y;
        return this;
    }

    // Overloaded operator for multiplication with a number with assignment
    public Vector2f multiply(float number) {
        x *= number;
        y *= number;
        return this;
    }

    // Normalize the vector
    public Vector2f normalize() {
        float len = length();
        assert (len > Mathematics.MACHINE_EPSILON);
        x /= len;
        y /= len;
        return this;
    }

    // Set all the values of the vector
    public final Vector2f set(float x, float y) {
        this.x = x;
        this.y = y;
        return this;
    }

    // Assignment operator
    public final Vector2f set(Vector2f vector) {
        x = vector.x;
        y = vector.y;
        return this;
    }

    // Return one unit orthogonal vector of the current vector
    public Vector2f setUnitOrthogonal() {
        float len = length();
        assert (len > Mathematics.MACHINE_EPSILON);
        return set(-y / len, x / len);
    }

    public Vector2f setX(float x) {
        this.x = x;
        return this;
    }

    public Vector2f setY(float y) {
        this.y = y;
        return this;
    }

    // Overloaded operator for substraction with assignment
    public Vector2f subtract(Vector2f vector) {
        x -= vector.x;
        y -= vector.y;
        return this;
    }

    // Set the vector to zero
    public final Vector2f zero() {
        x = 0.0f;
        y = 0.0f;
        return this;
    }

    @Override
    public int hashCode() {
        int hash = 3;
        hash = 53 * hash + Float.floatToIntBits(this.x);
        hash = 53 * hash + Float.floatToIntBits(this.y);
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
        final Vector2f other = (Vector2f) obj;
        if (Float.floatToIntBits(this.x) != Float.floatToIntBits(other.x)) {
            return false;
        }
        return Float.floatToIntBits(this.y) == Float.floatToIntBits(other.y);
    }

    @Override
    public String toString() {
        return "(x= " + x + ", y= " + y + ")";
    }

}