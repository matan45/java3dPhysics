package collisionDetection.util;

import collisionDetection.narrowPhase.gjk.GJKSupport;
import math.Vector3f;

public class CollisionUtil {

    public static Vector3f support(GJKSupport shape1, GJKSupport shape2, Vector3f direction) {
        Vector3f pointA = shape1.support(direction);
        Vector3f pointB = shape2.support(direction.negate());
        return pointA.sub(pointB);
    }
}
