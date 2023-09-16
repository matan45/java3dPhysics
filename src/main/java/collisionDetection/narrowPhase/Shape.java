package collisionDetection.narrowPhase;

import math.Quaternion;
import math.Vector3f;

public interface Shape {

    boolean isPointInside(Vector3f point);

    Vector3f closestPoint(Vector3f point);

    default void translate(Vector3f position) {
    }

    default void scale(Vector3f scale) {
    }

    default void rotate(Quaternion rotate) {
    }
}
