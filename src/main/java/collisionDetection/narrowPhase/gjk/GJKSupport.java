package collisionDetection.narrowPhase.gjk;

import collisionDetection.narrowPhase.Shape;
import math.Vector3f;

public interface GJKSupport extends Shape {

    Vector3f support(Vector3f  direction);
}
