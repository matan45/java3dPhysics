package math;

import static math.Const.EPSILON;

public class Maths {
    public static float barryCentric(Vector3f p1, Vector3f p2, Vector3f p3, Vector2f pos) {
        float det = (p2.z - p3.z) * (p1.x - p3.x) + (p3.x - p2.x) * (p1.z - p3.z);
        float l1 = ((p2.z - p3.z) * (pos.x - p3.x) + (p3.x - p2.x) * (pos.y - p3.z)) / det;
        float l2 = ((p3.z - p1.z) * (pos.x - p3.x) + (p1.x - p3.x) * (pos.y - p3.z)) / det;
        float l3 = 1.0f - l1 - l2;
        return l1 * p1.y + l2 * p2.y + l3 * p3.y;
    }

    public static Vector3f getRandomNumber(Vector3f min, Vector3f max) {
        return new Vector3f((float) ((Math.random() * (max.x - min.x)) + min.x),
                (float) ((Math.random() * (max.y - min.y)) + min.y),
                (float) ((Math.random() * (max.z - min.z)) + min.z));

    }

    public static Vector4f getRandomNumber(Vector4f min, Vector4f max) {
        return new Vector4f((float) ((Math.random() * (max.x - min.x)) + min.x),
                (float) ((Math.random() * (max.y - min.y)) + min.y),
                (float) ((Math.random() * (max.z - min.z)) + min.z),
                (float) ((Math.random() * (max.w - min.w)) + min.w));

    }

    public static Vector2f getRandomNumber(Vector2f min, Vector2f max) {
        return new Vector2f((float) ((Math.random() * (max.x - min.x)) + min.x),
                (float) ((Math.random() * (max.y - min.y)) + min.y));

    }

    public static float getRandomNumber(int min, int max) {
        return (float) ((Math.random() * (max - min)) + min);
    }

    public static float inverseSqrt(float r) {
        return 1.0f / (float) Math.sqrt(r);
    }

    public static float clamp(float value, float min, float max) {
        return (value < min) ? min : Math.min(value, max);
    }

    public static float interpretTo(float current, float target, float deltaTime, float interpreted) {
        // If no interp speed, jump to target value
        if (interpreted <= 0.f) {
            return target;
        }

        // Distance to reach
        final float dist = target - current;

        // If distance is too small, just set the desired location
        if (Math.sqrt(dist) < EPSILON) {
            return target;
        }

        // Delta Move, Clamp so we do not over shoot.
        final float deltaMove = dist * clamp(deltaTime * interpreted, 0.f, 1.f);

        return current + deltaMove;
    }


}
