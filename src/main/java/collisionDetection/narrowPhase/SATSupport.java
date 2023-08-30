package collisionDetection.narrowPhase;

import collisionDetection.primitive.Interval;
import math.Vector3f;

public interface SATSupport {

    Interval getInterval(Vector3f axis);
}
