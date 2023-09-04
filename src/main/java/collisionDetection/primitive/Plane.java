package collisionDetection.primitive;

import collisionDetection.narrowPhase.Shape;
import math.Const;
import math.Vector3f;

public class Plane implements Shape {
    private Vector3f normal; // Normal vector of the plane
    private float distance;  // Distance from the origin to the plane along the normal

    public Plane(Vector3f normal, float distance) {
        this.normal = normal.normalize(); // Normalize the normal vector
        this.distance = distance;
    }

    public boolean isPointInFront(Vector3f point) {
        return normal.dot(point) + distance > 0;
    }

    public Vector3f getNormal() {
        return normal;
    }

    public void setNormal(Vector3f normal) {
        this.normal = normal;
    }

    public float getDistance() {
        return distance;
    }

    public void setDistance(float distance) {
        this.distance = distance;
    }

    @Override
    public boolean isPointInside(Vector3f point) {
        // Calculate the signed distance from the point to the plane
        float signedDistance = point.dot(normal) - distance;

        // If the signed distance is very close to zero, consider the point to be on the plane
        return Math.abs(signedDistance) < Const.EPSILON;
    }

    @Override
    public Vector3f closestPoint(Vector3f point) {
        float dot = point.dot(normal);
        float dis = dot - distance;
        return point.sub(normal.mul(dis));
    }

    @Override
    public String toString() {
        return "Plane{" +
                "normal=" + normal +
                ", distance=" + distance +
                '}';
    }
}
