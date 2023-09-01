package collisionDetection.narrowPhase.gjk;

import collisionDetection.narrowPhase.collision_result.CollisionResult;
import math.Vector3f;

public class GJK {

    private static final int MAX_ITERATORS = 50;

    public static CollisionResult gjkCollision(GJKSupport shape1, GJKSupport shape2) {
        Simplex simplex = new Simplex();
        Vector3f direction = new Vector3f(1, 1, 1); // Initial search direction
        Vector3f pointA = shape1.support(direction);
        Vector3f pointB = shape2.support(direction.negate());

        simplex.addPoint(pointA.sub(pointB)); // Initial simplex

        for (int iteration = 0; iteration < MAX_ITERATORS; iteration++) {
            pointA = shape1.support(direction);
            pointB = shape2.support(direction.negate());

            Vector3f newPoint = pointA.sub(pointB);

            if (newPoint.dot(direction) < 0) {
                return EPA.epaCollisionResult(shape1, shape2, simplex); // No collision
            }

            simplex.addPoint(newPoint);

            // If the simplex has reached rank 3, then check for collision
            if (simplex.containsOrigin()) {
                return EPA.epaCollisionResult(shape1, shape2, simplex); // No collision
            }

        }

        return EPA.epaCollisionResult(shape1, shape2, simplex); // No collision
    }

}
