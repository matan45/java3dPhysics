package collisionDetection.primitive;

import collisionDetection.narrowPhase.Shape;
import collisionDetection.narrowPhase.gjk.GJKSupport;
import math.Quaternion;
import math.Vector3f;

import java.util.Objects;

public class Cylinder implements Shape, GJKSupport {
    private Vector3f center;
    private Vector3f upAxis;
    private float radius;
    private float height;

    public Cylinder(Vector3f center, float radius, float height) {
        this.center = center;
        this.radius = radius;
        this.height = height;
        this.upAxis = Vector3f.YAxis;
    }

    public Vector3f getUpAxis() {
        return upAxis;
    }

    public void setUpAxis(Vector3f upAxis) {
        this.upAxis = upAxis;
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
    public void translate(Vector3f position) {
        center.set(center.add(position));
    }

    @Override
    public void scale(Vector3f scale) {
        radius *= Math.max(scale.x, scale.z); // Scale the radius in X and Z axes
        height *= scale.y; // Scale the height along the Y axis
    }

    @Override
    public void rotate(Quaternion rotate) {
        // Rotate the upAxis
        upAxis.set(upAxis.rotate(rotate));
    }

    @Override
    public Vector3f support(Vector3f direction) {
        // Calculate the support point based on the cylinder's geometry and direction.

        // First, calculate the direction's projection on the cylinder's axis.
        Vector3f axis = upAxis; // Assuming the cylinder is aligned with the Y-axis.
        float projection = direction.dot(axis);

        // Calculate the support point on the cylinder's surface along the axis.
        float halfHeight = height / 2.0f;
        float clampedProjection = Math.min(halfHeight, Math.max(-halfHeight, projection));
        Vector3f supportOnAxis = axis.mul(clampedProjection);

        // Calculate the support point on the cylinder's circular top/bottom surface.
        Vector3f supportOnCircle = direction.sub(supportOnAxis).normalize().mul(radius);

        // Calculate the final support point by adding the components together.
        return center.add(supportOnAxis).add(supportOnCircle);
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
