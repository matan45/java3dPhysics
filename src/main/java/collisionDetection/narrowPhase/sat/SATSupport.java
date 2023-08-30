package collisionDetection.narrowPhase.sat;

import math.Vector3f;

public interface SATSupport {

    Interval getInterval(Vector3f axis);
}
