package collisionDetection.primitive;

import collisionDetection.narrowPhase.Shape;
import collisionDetection.narrowPhase.gjk.GJKSupport;
import collisionDetection.narrowPhase.sat.Interval;
import collisionDetection.narrowPhase.sat.SATSupport;
import math.Vector3f;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

public class Cylinder implements Shape, GJKSupport, SATSupport {
    private Vector3f center;
    private float radius;
    private float height;

    public Cylinder(Vector3f center, float radius, float height) {
        this.center = center;
        this.radius = radius;
        this.height = height;
    }

    public Vector3f getCenter() {
        return center;
    }

    public float getRadius() {
        return radius;
    }

    public float getHeight() {
        return height;
    }

    public void setCenter(Vector3f center) {
        this.center = center;
    }

    public void setRadius(float radius) {
        this.radius = radius;
    }

    public void setHeight(float height) {
        this.height = height;
    }

    @Override
    public boolean isPointInside(Vector3f point) {
        // Calculate the distance between the point and the cylinder's center in the x-z plane
        float distanceXZ = (float) Math.sqrt(Math.pow(point.x - center.x, 2) + Math.pow(point.z - center.z, 2));

        // Check if the point is within the radius and height bounds of the cylinder
        boolean isWithinRadius = distanceXZ <= radius;
        boolean isWithinHeight = point.y >= center.y && point.y <= center.y + height;

        // Return true if the point is within both radius and height bounds, false otherwise
        return isWithinRadius && isWithinHeight;
    }

    @Override
    public Vector3f closestPoint(Vector3f point) {
        // Calculate the direction from the cylinder's center to the given point
        Vector3f direction = new Vector3f(point.x - center.x, 0, point.z - center.z);

        // Clamp the direction's length to be within the cylinder's radius
        // Scale the normalized direction vector by the radius
        direction = direction.normalize().mul(radius);  // Normalize the direction vector

        // Calculate the closest point on the cylinder's axis
        Vector3f closestPointOnAxis = new Vector3f(center.x + direction.x, center.y, center.z + direction.z);

        // Calculate the closest point on the cylinder's surface
        float clampedY = Math.max(center.y, Math.min(center.y + height, point.y)); // Clamp y-coordinate

        return new Vector3f(closestPointOnAxis.x, clampedY, closestPointOnAxis.z);
    }

    @Override
    public Vector3f support(Vector3f direction) {
        // Compute the direction projected onto the axis of the cylinder
        Vector3f axis = Vector3f.YAxis; // Cylinder's axis, assuming it's aligned with the Y-axis
        float projection = direction.dot(axis);

        // Calculate the endpoint on the cylinder's axis
        Vector3f axisEndpoint = new Vector3f(center.x, center.y + height, center.z).add(axis.mul(projection));

        // Calculate the support point on the cylinder's surface
        return axisEndpoint.add(direction.normalize().mul(radius));
    }

    @Override
    public Interval getInterval(Vector3f axis) {
        // Calculate the projection of the center of the cylinder onto the axis
        float projectionCenter = center.dot(axis);

        // Calculate the half-length of the cylinder along the axis
        float halfLength = height / 2.0f;

        // Calculate the interval min and max
        float min = projectionCenter - halfLength;
        float max = projectionCenter + halfLength;

        return new Interval(min - radius, max + radius);
    }

    @Override
    public List<Vector3f> getAxis() {
        List<Vector3f> axes = new ArrayList<>();

        // Axis along the cylinder's central axis
        Vector3f cylinderAxis = Vector3f.ZAxis; // Assuming cylinder is aligned with the z-axis
        axes.add(cylinderAxis);

        // Perpendicular axes (choose any two perpendicular vectors)
        Vector3f arbitraryVector1 = Vector3f.XAxis;
        Vector3f arbitraryVector2 = Vector3f.YAxis;

        // Calculate the perpendicular axes based on the cylinder's orientation
        Vector3f perpendicularAxis1 = arbitraryVector1.cross(cylinderAxis);
        Vector3f perpendicularAxis2 = arbitraryVector2.cross(cylinderAxis);

        // Normalize the perpendicular axes
        axes.add(perpendicularAxis1.normalize());
        axes.add(perpendicularAxis2.normalize());

        return axes;
    }

    @Override
    public String toString() {
        return "Cylinder{" +
                "center=" + center +
                ", radius=" + radius +
                ", height=" + height +
                '}';
    }

    @Override
    public int hashCode() {
        return Objects.hash(center, radius, height);
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Cylinder cylinder = (Cylinder) o;
        return Float.compare(cylinder.radius, radius) == 0 && Float.compare(cylinder.height, height) == 0 && Objects.equals(center, cylinder.center);
    }
}
