package collisionDetection.primitive;

import collisionDetection.narrowPhase.Shape;
import math.Vector3f;

public class Ray implements Shape {

    private Vector3f origin;
    private Vector3f direction;

    public Ray(Vector3f origin, Vector3f direction) {
        this.origin = origin;
        this.direction = direction.normalize();
    }

    public Vector3f getOrigin() {
        return origin;
    }

    public void setOrigin(Vector3f origin) {
        this.origin = origin;
    }

    public Vector3f getDirection() {
        return direction;
    }

    public void setDirection(Vector3f direction) {
        this.direction = direction;
    }

    @Override
    public Vector3f closestPoint(Vector3f point) {
        float t = point.sub(origin).dot(direction);
        t = Math.max(t, 0.0f);
        return origin.add(direction.mul(t));
    }

    @Override
    public boolean isPointInside(Vector3f point) {
        // Calculate the vector from the ray's origin to the given point
        Vector3f rayToPoint = point.sub(origin);

        // Calculate the dot product between the ray direction and the vector to the point
        float dotProduct = rayToPoint.dot(direction);

        // If the dot product is greater than or equal to 0, the point is in the same direction as the ray.
        // This means the point is on or "inside" the ray.
        return dotProduct > 0.0f;
    }


    @Override
    public String toString() {
        return "Ray{" +
                "origin=" + origin +
                ", direction=" + direction +
                '}';
    }
}
