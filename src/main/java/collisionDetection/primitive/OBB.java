package collisionDetection.primitive;

import collisionDetection.narrowPhase.sat.Interval;
import math.Vector3f;

import java.util.Arrays;

public class OBB {

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
            test[6 + i * 3 + 1] =test[i].cross(test[4]);
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
        Interval projection1 = getInterval(axis, obb1);
        Interval projection2 = getInterval(axis, obb2);

        // Check for separation between the intervals
        return projection1.getMax() < projection2.getMin() || projection2.getMax() < projection1.getMin();
    }

    public static Interval getInterval(Vector3f axis, OBB obb) {
        float centerProjection = axis.x * obb.getCenter().x + axis.y * obb.getCenter().y + axis.z * obb.getCenter().z;

        // Calculate the half-length of the projection
        float halfLength =
                obb.getHalfExtents().x * Math.abs(axis.x) +
                        obb.getHalfExtents().y * Math.abs(axis.y) +
                        obb.getHalfExtents().z * Math.abs(axis.z);

        // Calculate the interval
        float min = centerProjection - halfLength;
        float max = centerProjection + halfLength;

        return new Interval(min, max);
    }

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
}
