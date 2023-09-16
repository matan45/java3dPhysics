package collisionDetection.narrowPhase.gjk;

import collisionDetection.util.CollisionUtil;
import math.Vector3f;

import static math.Const.GJK_EPA_MAX_ITERATORS;

public class GJK {

    private final EPA epa;

    public GJK() {
        this.epa = new EPA();
    }

    public boolean isCollide(GJKSupport shape1, GJKSupport shape2) {
        Simplex simplex = new Simplex();

        Vector3f support = CollisionUtil.support(shape1, shape2, new Vector3f(1, 0, 0));
        simplex.pushFront(support); // Initial simplex

        Vector3f direction = support.negate();

        for (int i = 0; i < GJK_EPA_MAX_ITERATORS; i++) {
            support = CollisionUtil.support(shape1, shape2, direction);

            if (support.dot(direction) < 0) {
                return false; // No collision
            }

            simplex.pushFront(support);

            // If the simplex has reached rank 3, then check for collision
            if (nextSimplex(simplex, direction)) {
                return true;
            }

        }

        return false; // No collision
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
            points.setValue(0, a);
            points.setSize(1);
            direction.set(ao);
        }

        return false;
    }

    private static boolean triangle(Simplex points, Vector3f direction) {
        Vector3f a = points.getPoint(0);// 1,1,1
        Vector3f b = points.getPoint(1);
        Vector3f c = points.getPoint(2);

        Vector3f ab = b.sub(a);
        Vector3f ac = c.sub(a);
        Vector3f ao = a.negate();

        Vector3f abc = ab.cross(ac);

        if (sameDirection(abc.cross(ac), ao)) {
            if (sameDirection(ac, ao)) {
                points.setValue(0, a);
                points.setValue(1, c);
                points.setSize(2);
                direction.sub(ac.cross(ao).cross(ac));
            } else {
                points.setValue(0, a);
                points.setValue(1, b);
                points.setSize(2);
                return line(points, direction);
            }
        } else {
            if (sameDirection(ab.cross(abc), ao)) {
                points.setValue(0, a);
                points.setValue(1, b);
                points.setSize(2);
                return line(points, direction);
            } else {
                if (sameDirection(abc, ao)) {
                    direction.set(abc);
                } else {
                    points.setValue(0, a);
                    points.setValue(1, c);
                    points.setValue(2, b);
                    points.setSize(3);
                    direction.set(abc.negate());
                }
            }
        }

        return false;
    }

    private static boolean tetrahedron(Simplex points, Vector3f direction) {
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
            points.setValue(0, a);
            points.setValue(1, b);
            points.setValue(2, c);
            points.setSize(3);
            return triangle(points, direction);
        }

        if (sameDirection(acd, ao)) {
            points.setValue(0, a);
            points.setValue(1, c);
            points.setValue(2, d);
            points.setSize(3);
            return triangle(points, direction);
        }

        if (sameDirection(adb, ao)) {
            points.setValue(0, a);
            points.setValue(1, d);
            points.setValue(2, b);
            points.setSize(3);
            return triangle(points, direction);
        }

        return true;
    }

    private static boolean sameDirection(Vector3f direction, Vector3f ao) {
        return direction.dot(ao) > 0;
    }

}
