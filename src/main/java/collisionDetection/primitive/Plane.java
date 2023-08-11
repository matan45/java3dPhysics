package collisionDetection.primitive;

import org.joml.Vector3f;

public class Plane {
    private Vector3f normal; // Normal vector of the plane
    private float distance;  // Distance from the origin to the plane along the normal

    public Plane(Vector3f normal, float distance) {
        this.normal = normal.normalize(); // Normalize the normal vector
        this.distance = distance;
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

    public Vector3f closestPoint(Vector3f point) {
        float dot = point.dot(normal);
        float dis = dot - distance;
        return point.sub(normal.mul(dis));
    }

    public static boolean isPlaneColliding(Plane plane1, Plane plane2) {
        Vector3f normal1 = plane1.getNormal();
        Vector3f normal2 = plane2.getNormal();

        // Check if the normals are parallel (i.e., planes are colliding)
        return normal1.normalize().equals(normal2.normalize());
    }

    @Override
    public String toString() {
        return "Plane{" +
                "normal=" + normal +
                ", distance=" + distance +
                '}';
    }
}
