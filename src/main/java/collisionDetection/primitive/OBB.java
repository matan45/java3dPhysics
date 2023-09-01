package collisionDetection.primitive;

import collisionDetection.narrowPhase.Shape;
import collisionDetection.narrowPhase.gjk.GJKSupport;
import collisionDetection.narrowPhase.sat.Interval;
import collisionDetection.narrowPhase.sat.SATSupport;
import math.Vector3f;

import java.util.Arrays;

public class OBB implements Shape, SATSupport, GJKSupport {

    private Vector3f center;
    private Vector3f[] axis;
    private Vector3f halfExtents;

    public OBB(Vector3f center, Vector3f halfExtents) {
        this.center = center;
        this.axis = new Vector3f[]{new Vector3f(1.0f, 0.0f, 0.0f), new Vector3f(0.0f, 1.0f, 0.0f), new Vector3f(0.0f, 0.0f, 1.0f)};
        this.halfExtents = halfExtents;
    }

    public Vector3f getCenter() {
        return center;
    }

    public void setCenter(Vector3f center) {
        this.center = center;
    }

    public Vector3f[] getAxis() {
        return axis;
    }

    public void setAxis(Vector3f[] axis) {
        this.axis = axis;
    }

    public Vector3f getHalfExtents() {
        return halfExtents;
    }

    public void setHalfExtents(Vector3f halfExtents) {
        this.halfExtents = halfExtents;
    }

    public Vector3f getEdge(int edgeIndex) {
        // Calculate the endpoints of the edge
        Vector3f startPoint = new Vector3f(center);
        startPoint.add(axis[edgeIndex]);

        Vector3f endPoint = new Vector3f(center);
        endPoint.add(axis[edgeIndex].mul(halfExtents.get(edgeIndex)));

        // Calculate the edge vector
        return endPoint.sub(startPoint);
    }

    public static boolean isOBBColliding(OBB obb1, OBB obb2) {
        // Combine axes from both OBBs
        Vector3f[] test = new Vector3f[18];

        test[0] = obb1.getAxis()[0];
        test[1] = obb1.getAxis()[1];
        test[2] = obb1.getAxis()[2];
        test[3] = obb2.getAxis()[0];
        test[4] = obb2.getAxis()[1];
        test[5] = obb2.getAxis()[2];

        for (int i = 0; i < 3; ++i) {
            test[6 + i * 3] = test[i].cross(test[3]);
            test[6 + i * 3 + 1] = test[i].cross(test[4]);
            test[6 + i * 3 + 2] = test[i].cross(test[5]);
        }

        // Include edge normals of both OBBs
        for (int i = 0; i < 3; ++i) {
            test[12 + i] = obb1.getEdge(i).normalize();
            test[15 + i] = obb2.getEdge(i).normalize();
        }

        // Check for separation along each axis
        for (Vector3f axis : test) {
            if (isAxisSeparating(axis, obb1, obb2)) {
                return false; // No collision along this axis
            }
        }

        return true; // No separation along any axis, collision detected
    }

    private static boolean isAxisSeparating(Vector3f axis, OBB obb1, OBB obb2) {
        // Project the OBBs onto the axis
        Interval projection1 = obb1.getInterval(axis);
        Interval projection2 = obb2.getInterval(axis);

        // Check for separation between the intervals
        return projection1.getMax() < projection2.getMin() || projection2.getMax() < projection1.getMin();
    }

    @Override
    public Interval getInterval(Vector3f axis) {
        float centerProjection = axis.x * getCenter().x + axis.y * getCenter().y + axis.z * getCenter().z;

        // Calculate the half-length of the projection
        float halfLength =
                getHalfExtents().x * Math.abs(axis.x) +
                        getHalfExtents().y * Math.abs(axis.y) +
                        getHalfExtents().z * Math.abs(axis.z);

        // Calculate the interval
        float min = centerProjection - halfLength;
        float max = centerProjection + halfLength;

        return new Interval(min, max);
    }

    @Override
    public boolean isPointInside(Vector3f point) {
        Vector3f localPoint = point.sub(center); // Transform point to local space
        for (int i = 0; i < 3; i++) {
            float distance = localPoint.dot(axis[i]);
            if (distance > halfExtents.get(i) || distance < -halfExtents.get(i)) {
                return false; // Point is outside along this axis
            }
        }
        return true; // Point is inside along all axes
    }

    @Override
    public Vector3f closestPoint(Vector3f point) {
        Vector3f result = center;
        Vector3f dir = point.sub(center);

        for (int i = 0; i < 3; ++i) {
            float distance = dir.dot(axis[i]);

            if (distance > halfExtents.get(i)) {
                distance = halfExtents.get(i);
            }
            if (distance < -halfExtents.get(i)) {
                distance = -halfExtents.get(i);
            }
            result = result.add(axis[i].mul(distance));
        }

        return result;
    }

    @Override
    public String toString() {
        return "OBB{" +
                "center=" + center +
                ", axis=" + Arrays.toString(axis) +
                ", halfExtents=" + halfExtents +
                '}';
    }

    @Override
    public Vector3f support(Vector3f direction) {
        Vector3f result = new Vector3f();

        for (int i = 0; i < 3; i++) {
            float projection = axis[i].dot(direction);
            result.add(new Vector3f(axis[i].mul(halfExtents.get(i) * ((projection >= 0) ? 1 : -1))));
        }

        result.add(center);
        return result;
    }
}
