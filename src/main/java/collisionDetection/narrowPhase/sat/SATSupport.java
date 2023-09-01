package collisionDetection.narrowPhase.sat;

import math.Vector3f;

import java.util.List;

public interface SATSupport {

    Interval getInterval(Vector3f axis);

    List<Vector3f> getAxis();
}
