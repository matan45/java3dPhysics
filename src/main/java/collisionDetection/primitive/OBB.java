package collisionDetection.primitive;

import collisionDetection.narrowPhase.Shape;
import collisionDetection.narrowPhase.gjk.GJKSupport;
import collisionDetection.narrowPhase.sat.Interval;
import collisionDetection.narrowPhase.sat.SATSupport;
import math.Quaternion;
import math.Vector3f;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;

public class OBB implements Shape, SATSupport, GJKSupport {

    private Vector3f center;
    private Vector3f[] axis;
    private Vector3f halfExtents;

    public OBB(Vector3f center, Vector3f halfExtents) {
        this.center = center;
        this.axis = new Vector3f[]{Vector3f.XAxis, Vector3f.YAxis, Vector3f.ZAxis};
        this.halfExtents = halfExtents;
    }

    public Vector3f getCenter() {
        return center;
    }

    public void setCenter(Vector3f center) {
        this.center = center;
    }

    @Override
    public List<Vector3f> getAxis() {
        return List.of(axis);
    }

    @Override
    public List<Vector3f> getVertices() {
        List<Vector3f> vertices = new ArrayList<>();

        // Calculate the eight vertices of the OBB.
        Vector3f[] localHalfExtents = new Vector3f[]{
                halfExtents.mul(-1f),
                halfExtents.mul(1f)
        };

        for (Vector3f axis : axis) {
            for (Vector3f halfExtent : localHalfExtents) {
                vertices.add(center.add(axis.mul(halfExtent)));
            }
        }

        return vertices;
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
    public Vector3f support(Vector3f direction) {
        // Transform the direction from world space to OBB local space
        Vector3f localDirection = new Vector3f(
                direction.dot(axis[0]),
                direction.dot(axis[1]),
                direction.dot(axis[2])
        );

        // Calculate the support point in local coordinates
        Vector3f localSupport = new Vector3f(
                Math.signum(localDirection.x) * halfExtents.x,
                Math.signum(localDirection.y) * halfExtents.y,
                Math.signum(localDirection.z) * halfExtents.z
        );

        // Calculate the world space support point
        return new Vector3f(
                axis[0].x * localSupport.x,
                axis[1].y * localSupport.y,
                axis[2].z * localSupport.z
        ).add(center);
    }

    public void translate(Vector3f translation) {
        center.set(center.add(translation));
    }

    public void scale(Vector3f scale) {
        halfExtents.set(halfExtents.mul(scale));
    }

    public void rotate(Quaternion rotation) {
        // Rotate each axis by the given quaternion
        for (int i = 0; i < axis.length; i++) {
            axis[i] = axis[i].rotate(rotation);
        }
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
    public int hashCode() {
        int result = Objects.hash(center, halfExtents);
        result = 31 * result + Arrays.hashCode(axis);
        return result;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        OBB obb = (OBB) o;
        return Objects.equals(center, obb.center) && Arrays.equals(axis, obb.axis) && Objects.equals(halfExtents, obb.halfExtents);
    }
}
