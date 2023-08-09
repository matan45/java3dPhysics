package collisionDetection.primitive;

import org.joml.Vector3f;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

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

    public static boolean isOBBColliding(OBB obb1, OBB obb2) {
        // Combine axes from both OBBs
        List<Vector3f> axes = new ArrayList<>();
        axes.addAll(Arrays.asList(obb1.getAxis()));
        axes.addAll(Arrays.asList(obb2.getAxis()));

        // Check for separation along each axis
        for (Vector3f axis : axes) {
            if (isAxisSeparating(axis, obb1, obb2)) {
                return false; // No collision along this axis
            }
        }

        return true; // No separation along any axis, collision detected
    }

    private static boolean isAxisSeparating(Vector3f axis, OBB obb1, OBB obb2) {
        // Project the OBBs onto the axis
        float projection1 = projectOntoAxis(axis, obb1);
        float projection2 = projectOntoAxis(axis, obb2);

        // Calculate the distance between the projections
        float distance = Math.abs(projection1 - projection2);

        // Calculate the total length of projections
        float totalLength = (obb1.getHalfExtents().x + obb2.getHalfExtents().x) * Math.abs(axis.x)
                + (obb1.getHalfExtents().y + obb2.getHalfExtents().y) * Math.abs(axis.y)
                + (obb1.getHalfExtents().z + obb2.getHalfExtents().z) * Math.abs(axis.z);

        // Check for separation
        return distance > totalLength;
    }

    public static float projectOntoAxis(Vector3f axis, OBB obb) {
        // Project the center of the OBB onto the axis
        return axis.x * obb.getCenter().x + axis.y * obb.getCenter().y + axis.z * obb.getCenter().z;
    }
}