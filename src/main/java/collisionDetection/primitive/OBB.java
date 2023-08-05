package collisionDetection.primitive;

import org.joml.Vector3f;

import java.util.Arrays;

public class OBB {
    private Vector3f center; // Center position of the OBB
    private Vector3f[] axes; // Array of three unit vectors representing the axes
    private float[] halfLengths; // Array of three half-lengths along each axis

    public OBB(Vector3f center, Vector3f[] axes, float[] halfLengths) {
        this.center = center;
        this.halfLengths = halfLengths;
        this.axes = axes;
    }

    public Vector3f[] getAxes() {
        return axes;
    }

    public void setAxes(Vector3f[] axes) {
        this.axes = axes;
    }

    public Vector3f getCenter() {
        return center;
    }

    public void setCenter(Vector3f center) {
        this.center = center;
    }

    public float[] getHalfLengths() {
        return halfLengths;
    }

    public void setHalfLengths(float[] halfLengths) {
        this.halfLengths = halfLengths;
    }

    public Vector3f[] getCorners() {
        Vector3f[] corners = new Vector3f[8];
        float[] halfLengths = getHalfLengths();

        // Generate corners by combining each axis's extreme points (min and max)
        for (int i = 0; i < 8; i++) {
            float x = center.x + axes[0].x * ((i & 1) == 0 ? halfLengths[0] : -halfLengths[0]);
            float y = center.y + axes[1].y * ((i & 2) == 0 ? halfLengths[1] : -halfLengths[1]);
            float z = center.z + axes[2].z * ((i & 4) == 0 ? halfLengths[2] : -halfLengths[2]);
            corners[i] = new Vector3f(x, y, z);
        }

        return corners;
    }


    public static CollisionResult isOBBCollidingWithOBB(OBB obb1, OBB obb2) {
        Vector3f[] obb1Corners = obb1.getCorners();
        Vector3f[] obb2Corners = obb2.getCorners();

        // Check for overlap on each axis
        float minDistance = Float.MAX_VALUE;
        Vector3f closestPoint1 = null;
        Vector3f closestPoint2 = null;
        boolean isColliding = true;

        for (int i = 0; i < 3; i++) {
            if (!overlapOnAxis(obb1Corners, obb2Corners, obb1.getAxes()[i])) {
                isColliding = false;
                break;
            }
            if (!overlapOnAxis(obb2Corners, obb1Corners, obb2.getAxes()[i])) {
                isColliding = false;
                break;
            }
        }

        if (isColliding) {
            // If there is overlap on all axes, calculate the closest points
            for (Vector3f corner1 : obb1Corners) {
                for (Vector3f corner2 : obb2Corners) {
                    float distanceSquared = corner1.distanceSquared(corner2);
                    if (distanceSquared < minDistance) {
                        minDistance = distanceSquared;
                        closestPoint1 = corner1;
                        closestPoint2 = corner2;
                    }
                }
            }
        } else {
            // Find the closest points between the two OBBs using their axes
            Vector3f[] axes = {obb1.getAxes()[0], obb1.getAxes()[1], obb1.getAxes()[2],
                    obb2.getAxes()[0], obb2.getAxes()[1], obb2.getAxes()[2]};
            for (Vector3f axis : axes) {
                Vector3f point1 = findSupportPoint(obb1Corners, axis);
                Vector3f point2 = findSupportPoint(obb2Corners, axis.negate());

                float distanceSquared = point1.distanceSquared(point2);
                if (distanceSquared < minDistance) {
                    minDistance = distanceSquared;
                    closestPoint1 = point1;
                    closestPoint2 = point2;
                }
            }
        }

        return new CollisionResult(isColliding, Math.sqrt(minDistance), closestPoint1, closestPoint2);
    }

    private static boolean overlapOnAxis(Vector3f[] corners1, Vector3f[] corners2, Vector3f axis) {
        if (corners1 == null || corners2 == null || corners1.length == 0 || corners2.length == 0) {
            return false;
        }

        float min1 = Float.MAX_VALUE;
        float max1 = Float.MIN_VALUE;
        float min2 = Float.MAX_VALUE;
        float max2 = Float.MIN_VALUE;

        for (Vector3f corner : corners1) {
            float projection = corner.dot(axis);
            min1 = Math.min(min1, projection);
            max1 = Math.max(max1, projection);
        }

        for (Vector3f corner : corners2) {
            float projection = corner.dot(axis);
            min2 = Math.min(min2, projection);
            max2 = Math.max(max2, projection);
        }

        float distance = Math.max(0, Math.max(max1 - min2, max2 - min1));
        float axisLength = axis.length();

        return distance < axisLength;
    }

    private static Vector3f findSupportPoint(Vector3f[] corners, Vector3f axis) {
        float maxProjection = Float.MIN_VALUE;
        Vector3f supportPoint = null;

        for (Vector3f corner : corners) {
            float projection = corner.dot(axis);
            if (projection > maxProjection) {
                maxProjection = projection;
                supportPoint = new Vector3f(corner); // Create a new Vector3f to avoid modifying the original corners
            }
        }

        if (supportPoint == null) {
            // If the support point is still null, use the center of the OBB as a fallback
            supportPoint = new Vector3f();
        }

        return supportPoint;
    }


    @Override
    public String toString() {
        return "OBB{" +
                "center=" + center +
                ", axes=" + Arrays.toString(axes) +
                ", halfLengths=" + Arrays.toString(halfLengths) +
                '}';
    }
}
