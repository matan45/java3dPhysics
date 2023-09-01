package collisionDetection.narrowPhase.gjk;

import math.Vector3f;

import static math.Const.GJK_EPA_MAX_ITERATORS;

public class GJK {

    public static boolean collision(GJKSupport shape1, GJKSupport shape2) {
        Simplex simplex = new Simplex();
        Vector3f direction = new Vector3f(1, 0, 0); // Initial search direction

        simplex.addPoint(support(shape1, shape2, direction)); // Initial simplex
        direction = direction.negate();

        for (int i = 0; i < GJK_EPA_MAX_ITERATORS; i++) {
            Vector3f newPoint = support(shape1, shape2, direction);

            if (newPoint.dot(direction) < 0) {
                return false; // No collision
            }

            simplex.addPoint(newPoint);

            // If the simplex has reached rank 3, then check for collision
            if (nextSimplex(simplex, direction)) {
                return true;
            }

        }

        return false; // No collision
    }

    private static Vector3f support(GJKSupport shape1, GJKSupport shape2, Vector3f direction) {
        Vector3f pointA = shape1.support(direction);
        Vector3f pointB = shape2.support(direction.negate());
        return pointA.sub(pointB);
    }

    private static boolean nextSimplex(Simplex points, Vector3f direction) {
        return switch (points.getSize()) {
            case 2 -> line(points, direction);
            case 3 -> triangle(points, direction);
            case 4 -> tetrahedron(points, direction);
            // never should be here
            default -> false;
        };

    }

    private static boolean line(Simplex points, Vector3f direction) {
        Vector3f a = points.getPoint(0);
        Vector3f b = points.getPoint(1);

        Vector3f ab = b.sub(a);
        Vector3f ao = a.negate();

        if (sameDirection(ab, ao)) {
            direction.set(ab.cross(ao).cross(ab));
        } else {
            points.setPoints(new Vector3f[]{a});
            direction.set(ao);
        }

        return false;
    }

    public static boolean triangle(Simplex points, Vector3f direction) {
        Vector3f a = points.getPoint(0);
        Vector3f b = points.getPoint(1);
        Vector3f c = points.getPoint(2);

        Vector3f ab = b.sub(a);
        Vector3f ac = c.sub(a);
        Vector3f ao = a.negate();

        Vector3f abc = ab.cross(ac);

        if (sameDirection(abc.cross(ac.cross(ao)), ao)) {
            if (sameDirection(ac, ao)) {
                points.setPoints(new Vector3f[]{a, c});
                direction.sub(ac.cross(ao).cross(ac));
            } else {
                points.setPoints(new Vector3f[]{a, b});
                return line(points, direction);
            }
        } else {
            if (sameDirection(ab.cross(abc), ao)) {
                points.setPoints(new Vector3f[]{a, b});
                return line(points, direction);
            } else {
                if (sameDirection(abc, ao)) {
                    direction.set(abc);
                } else {
                    points.setPoints(new Vector3f[]{a, c, b});
                    direction.set(abc.negate());
                }
            }
        }

        return false;
    }

    public static boolean tetrahedron(Simplex points, Vector3f direction) {
        Vector3f a = points.getPoint(0);
        Vector3f b = points.getPoint(1);
        Vector3f c = points.getPoint(2);
        Vector3f d = points.getPoint(3);

        Vector3f ab = b.sub(a);
        Vector3f ac = c.sub(a);
        Vector3f ad = d.sub(a);
        Vector3f ao = a.negate();

        Vector3f abc = ab.cross(ac);
        Vector3f acd = ac.cross(ad);
        Vector3f adb = ad.cross(ab);

        if (sameDirection(abc, ao)) {
            points.setPoints(new Vector3f[]{a, b, c});
            return triangle(points, direction);
        }

        if (sameDirection(acd, ao)) {
            points.setPoints(new Vector3f[]{a, c, d});
            return triangle(points, direction);
        }

        if (sameDirection(adb, ao)) {
            points.setPoints(new Vector3f[]{a, d, b});
            return triangle(points, direction);
        }

        return true;
    }

    private static boolean sameDirection(Vector3f direction, Vector3f ao) {
        return direction.dot(ao) > 0;
    }

}
