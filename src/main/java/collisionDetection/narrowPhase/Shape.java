package collisionDetection.narrowPhase;

import math.Vector3f;

public interface Shape {

    boolean isPointInside(Vector3f point);

    Vector3f closestPoint(Vector3f point);
}
