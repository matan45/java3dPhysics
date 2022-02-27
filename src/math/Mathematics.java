package math;

public class Mathematics {
    private Mathematics(){}
    // Smallest decimal value (negative)
    public static final float DECIMAL_SMALLEST = Float.MIN_VALUE;

    // Maximum decimal value
    public static final float DECIMAL_LARGEST = Float.MAX_VALUE;

    // Machine epsilon
    public static final float MACHINE_EPSILON = 0.000001f;

    // Pi constant
    public static final float PI = 3.14159265f;

    // 2*Pi constant
    public static final float PI_TIMES_2 = 6.28318530f;

    // Function to test if two real numbers are (almost) equal
    // We test if two numbers a and b are such that (a-b) are in [-EPSILON; EPSILON]
    public static boolean ApproxEqual(float a, float b, float epsilon) {
        float difference = a - b;
        return (difference < epsilon && difference > -epsilon);
    }

    public static float ArcCos(float radians) {
        return (float) StrictMath.acos(radians);
    }

    public static float ArcSin(float radians) {
        return (float) StrictMath.asin(radians);
    }

    public static float ArcTan2(float a, float b) {
        return (float) StrictMath.atan2(a, b);
    }

    // Function that returns the result of the "value" clamped by
    // two others values "lowerLimit" and "upperLimit"
    public static float Clamp(float value, float lowerLimit, float upperLimit) {
        assert (lowerLimit <= upperLimit);
        return Math.min(Math.max(value, lowerLimit), upperLimit);
    }

    public static float Cos(float radians) {
        return (float) StrictMath.cos(radians);
    }

    public static float Sin(float radians) {
        return (float) StrictMath.sin(radians);
    }

    public static float Sqrt(float a) {
        return (float) StrictMath.sqrt(a);
    }

    public static float Tan(float radians) {
        return (float) StrictMath.tan(radians);
    }

    // Encodes a floating value into a integer value in order to work with integer
    // comparisons in the Sweep-And-Prune algorithm, for performance.
    // The main issue when encoding a floating number into an integer is keeping
    // the sorting order. This is a problem for negative float numbers.
    // This article describes how to solve this issue: http://www.stereopsis.com/radix.html
    public static long encodeFloatIntoInteger(float number) {
        long intNumber = (long) Float.floatToIntBits(number) & 0xFFFFFFFFl;
        if ((intNumber & 0x80000000l) == 0x80000000l) {
            intNumber = ~intNumber & 0xFFFFFFFFl;
        } else {
            intNumber |= 0x80000000l;
        }
        return intNumber;
    }
}
