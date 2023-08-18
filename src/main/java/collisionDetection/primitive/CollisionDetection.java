package collisionDetection.primitive;


import math.Vector3f;

public class CollisionDetection {
    public static boolean isSphereCollidingWithAABB(Sphere sphere, AABB aabb) {
        Vector3f closestPoint = aabb.closestPoint(sphere.getCenter());
        float distSq = sphere.getCenter().sub(closestPoint).lengthSquared();
        float radiusSq = sphere.getRadius() * sphere.getRadius();
        return distSq < radiusSq;
    }

    public static boolean isSphereCollidingWithPlane(Sphere sphere, Plane plane) {
        Vector3f closestPoint = plane.closestPoint(sphere.getCenter());
        float distSq = sphere.getCenter().sub(closestPoint).lengthSquared();
        float radiusSq = sphere.getRadius() * sphere.getRadius();
        return distSq < radiusSq;
    }

    public static boolean isSphereCollidingWithTriangle(Sphere sphere, Triangle triangle) {
        Vector3f closestPoint = triangle.closestPoint(sphere.getCenter());
        float distSq = sphere.getCenter().sub(closestPoint).lengthSquared();
        float radiusSq = sphere.getRadius() * sphere.getRadius();
        return distSq < radiusSq;
    }

    public static boolean isSphereCollidingWithOBB(Sphere sphere, OBB obb) {
        Vector3f closestPoint = obb.closestPoint(sphere.getCenter());
        float distSq = sphere.getCenter().sub(closestPoint).lengthSquared();
        float radiusSq = sphere.getRadius() * sphere.getRadius();
        return distSq < radiusSq;
    }

    public static boolean isAABBCollidingWithOBB(AABB aabb, OBB obb) {
        // Combine axes from OBB and aabb
        Vector3f[] test = new Vector3f[18];

        test[0] = new Vector3f(1, 0, 0);
        test[1] = new Vector3f(0, 1, 0);
        test[2] = new Vector3f(0, 0, 1);
        test[3] = obb.getAxis()[0];
        test[4] = obb.getAxis()[1];
        test[5] = obb.getAxis()[2];

        for (int i = 0; i < 3; ++i) {
            test[6 + i * 3] = test[i].cross(test[3]);
            test[6 + i * 3 + 1] = test[i].cross(test[4]);
            test[6 + i * 3 + 2] = test[i].cross(test[5]);
        }

        // Include edge normals of both OBBs
        for (int i = 0; i < 3; ++i) {
            test[12 + i] = obb.getEdge(i).normalize();
            test[15 + i] = obb.getEdge(i).normalize();
        }

        // Check for separation along each axis
        for (Vector3f axis : test) {
            if (isAxisSeparating(axis, aabb, obb)) {
                return false; // No collision along this axis
            }
        }

        return true; // No separation along any axis, collision detected
    }

    private static boolean isAxisSeparating(Vector3f axis, AABB aabb, OBB obb) {
        // Project the OBBs onto the axis
        Interval projection1 = AABB.getInterval(axis, aabb);
        Interval projection2 = OBB.getInterval(axis, obb);

        // Check for separation between the intervals
        return projection1.getMax() < projection2.getMin() || projection2.getMax() < projection1.getMin();
    }

    public static boolean isSphereCollidingWithCapsule(Sphere sphere, Capsule capsule) {
        // Calculate the squared distance between sphere center and capsule start and end points
        float sqDistToStart = sphere.getCenter().sub(capsule.getStart()).lengthSquared();
        float sqDistToEnd = sphere.getCenter().sub(capsule.getEnd()).lengthSquared();

        // Check if sphere is colliding with the capsule's cylindrical part
        if (sqDistToStart <= sphere.getRadius() * sphere.getRadius() || sqDistToEnd <= sphere.getRadius() * sphere.getRadius()) {
            return true; // Sphere collides with the end cap of the capsule
        }

        // Calculate the closest point on the capsule axis to the sphere center
        Vector3f capsuleDir = capsule.getEnd().sub(capsule.getStart());
        Vector3f sphereToCapsule = sphere.getCenter().sub(capsule.getStart());
        float t = sphereToCapsule.dot(capsuleDir) / capsuleDir.lengthSquared();
        t = Math.max(0.0f, Math.min(1.0f, t));
        Vector3f closestPoint = capsule.getStart().add(capsuleDir.mul(t));

        // Calculate the squared distance between sphere center and closest point on the capsule axis
        float sqDistToAxis = sphere.getCenter().sub(closestPoint).lengthSquared();

        // Check if sphere is colliding with the cylindrical part of the capsule
        return sqDistToAxis <= sphere.getRadius() * sphere.getRadius();
    }

    public static boolean isSphereCollidingWithCylinder(Sphere sphere, Cylinder cylinder) {
        // Calculate the squared distance between sphere center and cylinder center (in xz-plane)
        float sqDistToCenterXZ = (sphere.getCenter().x - cylinder.getCenter().x) * (sphere.getCenter().x - cylinder.getCenter().x) +
                (sphere.getCenter().z - cylinder.getCenter().z) * (sphere.getCenter().z - cylinder.getCenter().z);

        // Check if sphere is colliding with the circular top and bottom caps of the cylinder
        if (sphere.getCenter().y <= cylinder.getCenter().y + cylinder.getHeight() / 2.0f &&
                sphere.getCenter().y >= cylinder.getCenter().y - cylinder.getHeight() / 2.0f) {
            if (sqDistToCenterXZ <= cylinder.getRadius() * cylinder.getRadius() ||
                    sqDistToCenterXZ <= (cylinder.getRadius() - sphere.getRadius()) * (cylinder.getRadius() - sphere.getRadius())) {
                return true; // Sphere collides with the top or bottom cap of the cylinder
            }
        }

        // Check if sphere's center is within the y-range of the cylinder's body
        float deltaY = Math.abs(sphere.getCenter().y - cylinder.getCenter().y);
        float yCollisionThreshold = cylinder.getHeight() / 2.0f + sphere.getRadius();
        if (deltaY <= yCollisionThreshold) {
            // Calculate the squared distance between sphere center and the cylinder axis in the xz-plane
            Vector3f cylinderCenterXZ = new Vector3f(cylinder.getCenter().x, 0.0f, cylinder.getCenter().z);
            Vector3f sphereCenterXZ = new Vector3f(sphere.getCenter().x, 0.0f, sphere.getCenter().z);
            float sqDistToAxisXZ = sphereCenterXZ.sub(cylinderCenterXZ).lengthSquared();

            // Check if sphere is colliding with the cylindrical part of the cylinder
            return sqDistToAxisXZ <= (cylinder.getRadius() + sphere.getRadius()) * (cylinder.getRadius() + sphere.getRadius());
        }

        return false;
    }

    public static boolean isPlaneCollidingWithAABB(Plane plane, AABB aabb) {
        return false;
    }

    public static boolean isCapsuleCollidingWithAABB(Capsule capsule, AABB aabb) {
        return false;
    }

    public static boolean isCylinderCollidingWithAABB(Cylinder cylinder, AABB aabb) {
        return false;
    }
    public static boolean isCylinderCollidingWithTriangle(Cylinder cylinder, Triangle triangle) {
        return false;
    }
    public static boolean isCylinderCollidingWithPlane(Cylinder cylinder, Plane plane) {
        return false;
    }
    public static boolean isCylinderCollidingWithOBB(Cylinder cylinder, OBB obb) {
        return false;
    }

    public static boolean isTriangleCollidingWithAABB(Triangle triangle, AABB aabb){
        return false;
    }
    public static boolean isTriangleCollidingWithPlane(Triangle triangle, Plane plane){
        return false;
    }
    public static boolean isCapsuleCollidingWithCylinder(Capsule capsule, Cylinder cylinder){
        return false;
    }
    public static boolean isCapsuleCollidingWithTriangle(Capsule capsule, Triangle triangle){
        return false;
    }
    public static boolean isCapsuleCollidingWithOBB(Capsule capsule, OBB obb){
        return false;
    }
    public static boolean isCapsuleCollidingWithPlane(Capsule capsule, Plane plane){
        return false;
    }


}
