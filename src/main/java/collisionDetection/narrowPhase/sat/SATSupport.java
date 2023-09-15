package collisionDetection.narrowPhase.sat;

import collisionDetection.narrowPhase.Shape;
import math.Vector3f;

import java.util.List;

public interface SATSupport extends Shape {

    Interval getInterval(Vector3f axis);

    List<Vector3f> getAxis();

    List<Vector3f> getVertices();
}
