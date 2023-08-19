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
        // Calculate the half-extents of the AABB
        Vector3f halfExtents = aabb.getMax().sub(aabb.getMin()).div(2.0f);

        // Calculate the center of the AABB
        Vector3f center = aabb.getMin().add(halfExtents);

        // Calculate the distance from the center of the AABB to the plane
        float distanceToPlane = center.dot(plane.getNormal()) - plane.getDistance();

        // Calculate the projection of the half-extents onto the plane's normal
        float projection = halfExtents.x * Math.abs(plane.getNormal().x)
                + halfExtents.y * Math.abs(plane.getNormal().y)
                + halfExtents.z * Math.abs(plane.getNormal().z);

        // Check if the AABB is on the opposite side of the plane's normal
        // compared to the plane's distance
        return Math.abs(distanceToPlane) <= projection;
    }
    public static boolean isCapsuleCollidingWithAABB(Capsule capsule, AABB aabb) {
        // Calculate squared radius for efficient distance comparison
        float radiusSquared = capsule.getRadius() * capsule.getRadius();

        // Calculate squared distance from the start of the capsule to the AABB
        float distanceToStartSquared = aabb.closestPoint(capsule.getStart()).lengthSquared();

        // Calculate squared distance from the end of the capsule to the AABB
        float distanceToEndSquared = aabb.closestPoint(capsule.getEnd()).lengthSquared();

        // If both the start and end of the capsule are outside the AABB and
        // their distances are greater than the squared radius, they're not colliding
        if (distanceToStartSquared > radiusSquared && distanceToEndSquared > radiusSquared) {
            return false;
        }

        // Otherwise, at least one of the capsule's ends is within the AABB or their
        // distances are within the radius, meaning they might be colliding

        // Calculate the closest point on the capsule axis to the AABB
        Vector3f closestPointOnAxis = aabb.closestPointOnSegmentToAABB(capsule.getStart(), capsule.getEnd());

        // Calculate the squared distance from the closest point on the axis to the AABB
        float distanceToAxisSquared = aabb.closestPoint(closestPointOnAxis).lengthSquared();

        // If the squared distance to the axis is greater than the radius squared,
        // the capsule is not colliding with the AABB
        return distanceToAxisSquared <= radiusSquared;
    }

    public static boolean isCylinderCollidingWithAABB(Cylinder cylinder, AABB aabb) {
        // Calculate squared radius for efficient distance comparison
        float radiusSquared = cylinder.getRadius() * cylinder.getRadius();

        // Calculate the squared distance from the center of the cylinder to the AABB
        float distanceToCenterSquared = aabb.closestPoint(cylinder.getCenter()).lengthSquared();

        // If the squared distance is greater than the cylinder's squared height and the squared radius,
        // and the center of the cylinder is outside the AABB, they're not colliding
        if (distanceToCenterSquared > cylinder.getHeight() * cylinder.getHeight() &&
                distanceToCenterSquared > radiusSquared) {
            return false;
        }

        // Otherwise, the center of the cylinder is within the AABB or their distances are within the
        // cylinder's height and radius, meaning they might be colliding

        // Calculate the projection of the cylinder's axis onto the AABB's axes
        float projectionX = Math.abs(cylinder.getCenter().x - Math.max(aabb.getMin().x, Math.min(cylinder.getCenter().x, aabb.getMax().x)));
        float projectionZ = Math.abs(cylinder.getCenter().z - Math.max(aabb.getMin().z, Math.min(cylinder.getCenter().z, aabb.getMax().z)));

        // If the projections are less than the cylinder's radius and the cylinder's height is greater than the
        // distance between the AABB's min and max along the Y-axis, they might be colliding
        return projectionX <= cylinder.getRadius() && projectionZ <= cylinder.getRadius() &&
                cylinder.getHeight() > aabb.getMax().y - aabb.getMin().y;
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
