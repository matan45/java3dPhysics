package math;

import collisionDetection.primitive.Triangle;

public class Maths {
    public static float barryCentric(Vector3f p1, Vector3f p2, Vector3f p3, Vector2f pos) {
        float det = (p2.z - p3.z) * (p1.x - p3.x) + (p3.x - p2.x) * (p1.z - p3.z);
        float l1 = ((p2.z - p3.z) * (pos.x - p3.x) + (p3.x - p2.x) * (pos.y - p3.z)) / det;
        float l2 = ((p3.z - p1.z) * (pos.x - p3.x) + (p1.x - p3.x) * (pos.y - p3.z)) / det;
        float l3 = 1.0f - l1 - l2;
        return l1 * p1.y + l2 * p2.y + l3 * p3.y;
    }

    public static float getRandomNumber(int min, int max) {
        return (float) ((Math.random() * (max - min)) + min);
    }

    public static Vector3f barycentric(Vector3f p, Triangle t) {
        Vector3f ap = p.sub(t.getVertex1());
        Vector3f bp = p.sub(t.getVertex2());
        Vector3f cp = p.sub(t.getVertex3());

        Vector3f ab = t.getVertex2().sub(t.getVertex1());
        Vector3f ac = t.getVertex3().sub(t.getVertex1());
        Vector3f bc = t.getVertex3().sub(t.getVertex2());
        Vector3f cb = t.getVertex2().sub(t.getVertex3());
        Vector3f ca = t.getVertex1().sub(t.getVertex3());

        Vector3f v = ab.sub(Vector3f.project(ab, cb));
        float a = 1.0f - (v.dot(ap) / v.dot(ab));

        v = bc.sub(Vector3f.project(bc, ac));
        float b = 1.0f - (v.dot(bp) / v.dot(bc));

        v = ca.sub(Vector3f.project(ca, ab));
        float c = 1.0f - (v.dot(cp) / v.dot(ca));

        return new Vector3f(a, b, c);
    }
}
