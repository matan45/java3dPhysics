package collisionDetection.primitive;


import collisionDetection.narrowPhase.sat.Interval;
import math.Vector3f;

import java.util.List;

import static math.Const.EPSILON;

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
        test[3] = obb.getAxis().get(0);
        test[4] = obb.getAxis().get(1);
        test[5] = obb.getAxis().get(2);

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
        Interval projection1 = aabb.getInterval(axis);
        Interval projection2 = obb.getInterval(axis);

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
        Vector3f cylinderTop = cylinder.getCenter().add(new Vector3f(0, cylinder.getHeight() / 2, 0));
        Vector3f cylinderBottom = cylinder.getCenter().add(new Vector3f(0, cylinder.getHeight() / 2, 0));

        // Calculate triangle's normal and constant term for the plane equation
        Vector3f triangleNormal = triangle.calculateTriangleNormal();
        float triangleConstant = -triangleNormal.dot(triangle.getVertex1());

        // Check if cylinder cap collides with triangle plane
        float distanceToTopCap = calculateDistanceToPlane(cylinderTop, triangleNormal, triangleConstant);
        float distanceToBottomCap = calculateDistanceToPlane(cylinderBottom, triangleNormal, triangleConstant);

        if (Math.abs(distanceToTopCap) < cylinder.getRadius() || Math.abs(distanceToBottomCap) < cylinder.getRadius()) {
            return true;
        }

        // Check cylinder side collision
        return isCylinderSideCollidingWithTriangle(cylinder, triangle);
    }

    private static boolean isCylinderSideCollidingWithTriangle(Cylinder cylinder, Triangle triangle) {
        // Calculate the triangle's normal
        Vector3f triangleNormal = triangle.calculateTriangleNormal();

        // Calculate the projection of the cylinder onto the triangle's plane
        Vector3f cylinderProjectionCenter = new Vector3f(cylinder.getCenter().x, triangle.getVertex1().y, cylinder.getCenter().z);

        // Project the cylinder's top and bottom circles onto the triangle's plane
        Vector3f projectedTopCenter = cylinderProjectionCenter.add(triangleNormal.mul(cylinder.getRadius()));
        Vector3f projectedBottomCenter = cylinderProjectionCenter.sub(triangleNormal.mul(cylinder.getRadius()));

        // Check if the projected circles intersect the triangle
        return isCircleIntersectingTriangle(projectedTopCenter, cylinder.getRadius(), triangle) ||
                isCircleIntersectingTriangle(projectedBottomCenter, cylinder.getRadius(), triangle);
    }

    private static boolean isCircleIntersectingTriangle(Vector3f circleCenter, float radius, Triangle triangle) {
        // Check if any vertex of the triangle is inside the circle
        if (isPointInsideCircle(circleCenter, radius, triangle.getVertex1()) ||
                isPointInsideCircle(circleCenter, radius, triangle.getVertex2()) ||
                isPointInsideCircle(circleCenter, radius, triangle.getVertex3())) {
            return true;
        }

        // Check if any triangle edge intersects the circle
        return isSegmentIntersectingCircle(triangle.getVertex1(), triangle.getVertex2(), circleCenter, radius) ||
                isSegmentIntersectingCircle(triangle.getVertex2(), triangle.getVertex3(), circleCenter, radius) ||
                isSegmentIntersectingCircle(triangle.getVertex3(), triangle.getVertex1(), circleCenter, radius);
    }

    private static boolean isPointInsideCircle(Vector3f circleCenter, float radius, Vector3f point) {
        float distanceSquared = point.sub( circleCenter).lengthSquared();
        return distanceSquared <= radius * radius;
    }

    private static boolean isSegmentIntersectingCircle(Vector3f startPoint, Vector3f endPoint, Vector3f circleCenter, float radius) {
        // Perform intersection test between a line segment and a circle
        // This is a simplified version and may not handle all cases
        // A more accurate algorithm like the Bresenham algorithm should be used

        // Calculate the closest point on the segment to the circle center
        Vector3f segmentDirection = endPoint.sub(startPoint);
        Vector3f toCircleCenter = circleCenter.sub(startPoint);
        float t = toCircleCenter.dot(segmentDirection) / segmentDirection.lengthSquared();
        t = Math.max(0, Math.min(1, t)); // Clamp t to the [0, 1] interval
        Vector3f closestPoint = startPoint.add(segmentDirection.mul(t));

        // Check if the closest point is within the circle's radius
        return isPointInsideCircle(circleCenter, radius, closestPoint);
    }

    private static float calculateDistanceToPlane(Vector3f point, Vector3f normal, float constant) {
        return normal.dot(point) + constant / normal.length();
    }

    public static boolean isCylinderCollidingWithPlane(Cylinder cylinder, Plane plane) {
        // Calculate the projection of the cylinder's center onto the plane's normal
        float distanceToPlane = cylinder.getCenter().dot(plane.getNormal()) - plane.getDistance();

        // Calculate the distance between the center of the cylinder's top cap and the plane
        float topCapDistance = Math.abs(distanceToPlane) - cylinder.getHeight() / 2.0f;

        // Calculate the distance between the center of the cylinder's bottom cap and the plane
        float bottomCapDistance = Math.abs(distanceToPlane) + cylinder.getHeight() / 2.0f;

        // If both the top and bottom caps are on the same side of the plane's normal, there's no collision
        if (topCapDistance > cylinder.getRadius() && bottomCapDistance > cylinder.getRadius()) {
            return false;
        }

        // If the center of the cylinder is on the plane or the top/bottom caps intersect the plane, there's a collision
        return Math.abs(distanceToPlane) <= cylinder.getRadius();
    }

    public static boolean isCapsuleCollidingWithPlane(Capsule capsule, Plane plane) {
        // Calculate the direction of the capsule's axis
        Vector3f capsuleAxis = capsule.getEnd().sub(capsule.getStart());

        // Calculate the projection of the capsule's axis onto the plane's normal
        float projection = capsuleAxis.dot(plane.getNormal());

        // If the capsule's axis is almost parallel to the plane, there's no collision
        if (Math.abs(projection) < EPSILON) {
            return false;
        }

        // Calculate the distance from the start of the capsule to the plane
        float distanceToStart = (capsule.getStart().dot(plane.getNormal()) - plane.getDistance()) / projection;

        // Calculate the distance from the end of the capsule to the plane
        float distanceToEnd = (capsule.getEnd().dot(plane.getNormal()) - plane.getDistance()) / projection;

        // If both the start and end of the capsule are on the same side of the plane, there's no collision
        return (!(distanceToStart < 0) || !(distanceToEnd < 0)) && (!(distanceToStart > 1) || !(distanceToEnd > 1));

    }

    public static boolean isCapsuleCollidingWithCylinder(Capsule capsule, Cylinder cylinder) {
        // Check if either of the capsule's ends is colliding with the cylinder's caps
        boolean startColliding = isPointCollidingWithCylinderCap(capsule.getStart(), cylinder);
        boolean endColliding = isPointCollidingWithCylinderCap(capsule.getEnd(), cylinder);

        // Check if the capsule's body is colliding with the cylinder's body
        boolean bodyColliding = cylinder.isSegmentCollidingWithCylinderBody(capsule.getStart(), capsule.getEnd());

        // If any of the capsule's ends are colliding with the cylinder's caps or the capsule's body
        // is colliding with the cylinder's body, they are colliding
        return startColliding || endColliding || bodyColliding;
    }

    private static boolean isPointCollidingWithCylinderCap(Vector3f point, Cylinder cylinder) {
        // Calculate the distance between the point and the cylinder's center in the XZ plane
        float distanceXZ = (float) Math.sqrt((point.x - cylinder.getCenter().x) * (point.x - cylinder.getCenter().x) +
                (point.z - cylinder.getCenter().z) * (point.z - cylinder.getCenter().z));

        // Check if the point is within the cylinder's cap radius and its Y coordinate is within the cap's height
        return distanceXZ <= cylinder.getRadius() && Math.abs(point.y - cylinder.getCenter().y) <= cylinder.getHeight() / 2.0f;
    }

    public static boolean isCylinderCollidingWithOBB(Cylinder cylinder, OBB obb) {
        // Calculate squared distance between cylinder.center and obb.center
        float dx = obb.getCenter().x - cylinder.getCenter().x;
        float dy = obb.getCenter().y - cylinder.getCenter().y;
        float dz = obb.getCenter().z - cylinder.getCenter().z;
        float distanceSquared = dx * dx + dy * dy + dz * dz;

        // Calculate the maximum extent sum
        float maxExtentSum = cylinder.getRadius() + obb.getHalfExtents().x + obb.getHalfExtents().y + obb.getHalfExtents().z;

        // Broad Phase Collision Detection
        if (distanceSquared > maxExtentSum * maxExtentSum) {
            // Shapes are too far apart, they can't collide
            return false;
        }

        // Perform Separating Axis Test for Cylinder-OBB
        if (!cylinderOBBIntersect(cylinder, obb)) {
            return false; // No collision
        }

        // Perform Caps and Faces Test
        return cylinderOBBCapsFacesIntersect(cylinder, obb); // No collision
    }
    private static boolean cylinderOBBIntersect(Cylinder cylinder, OBB obb) {
        Vector3f cylinderAxis = new Vector3f(0, 1, 0); // Assuming the cylinder's axis is along the y-axis
        Vector3f cylinderToOBB = obb.getCenter().sub(cylinder.getCenter());

        // Project the distance vector onto the cylinder's axis
        float distance = cylinderToOBB.dot(cylinderAxis);

        // Calculate the distance from the cylinder axis to the OBB center
        Vector3f projectedOBB = cylinderToOBB.sub(cylinderAxis.mul(distance));
        float projectedOBBLength = projectedOBB.length();

        // Calculate the combined radius of the cylinder and OBB projected onto the cylinder's axis
        float combinedRadius = cylinder.getRadius() + projectedOBBLength;

        // Check if the distance between the projected OBB and cylinder axis is less than the combined radius
        if (projectedOBBLength > combinedRadius) {
            // Separating axis found
            return false;
        }

        // Check for separation along OBB's local axes
        for (Vector3f axis : obb.getAxis()) {
            float obbProjection = obb.getHalfExtents().x * axis.dot(projectedOBB)
                    + obb.getHalfExtents().y * Math.abs(axis.dot(cylinderAxis))
                    + obb.getHalfExtents().z * Math.abs(axis.dot(cylinderAxis));

            if (Math.abs(distance) > obbProjection + cylinder.getRadius()) {
                // Separating axis found
                return false;
            }
        }

        return true; // No separating axis found, shapes intersect
    }

    private static boolean cylinderOBBCapsFacesIntersect(Cylinder cylinder, OBB obb) {

        // Calculate the distances from cylinder caps to OBB faces
        float distanceTopCap = Math.abs(obb.getAxis().get(1).dot(cylinder.getCenter().sub(obb.getCenter()))) + obb.getHalfExtents().y;
        float distanceBottomCap = Math.abs(obb.getAxis().get(1).dot(cylinder.getCenter().sub(obb.getCenter()))) - obb.getHalfExtents().y;

        // Check if cylinder top cap intersects with OBB top face
        if (distanceTopCap > cylinder.getHeight() * 0.5f) {
            return false;
        }

        // Check if cylinder bottom cap intersects with OBB bottom face
        return !(distanceBottomCap > cylinder.getHeight() * 0.5f);// Caps and faces intersect
    }

    public static boolean isTriangleCollidingWithAABB(Triangle triangle, AABB aabb) {

        // Check if any of the triangle's vertices are inside the AABB
        if (isPointInsideAABB(triangle.getVertex1(), aabb) ||
                isPointInsideAABB(triangle.getVertex2(), aabb) ||
                isPointInsideAABB(triangle.getVertex3(), aabb)) {
            return true;
        }

        // Check if the triangle intersects any of the AABB's edges
        return isTriangleEdgeIntersectingAABB(triangle.getVertex1(), triangle.getVertex2(), aabb) ||
                isTriangleEdgeIntersectingAABB(triangle.getVertex2(), triangle.getVertex3(), aabb) ||
                isTriangleEdgeIntersectingAABB(triangle.getVertex1(), triangle.getVertex3(), aabb);
    }
    public static boolean isPointInsideAABB(Vector3f point, AABB aabb) {
        return point.x >= aabb.getMin().x && point.x <= aabb.getMax().x &&
                point.y >= aabb.getMin().y && point.y <= aabb.getMax().y &&
                point.z >= aabb.getMin().z && point.z <= aabb.getMax().z;
    }

    public static boolean isTriangleEdgeIntersectingAABB(Vector3f vertex1, Vector3f vertex2, AABB aabb) {
        // Check if either vertex of the edge is inside the AABB
        if (isPointInsideAABB(vertex1, aabb) || isPointInsideAABB(vertex2, aabb)) {
            return true;
        }

        // Calculate the direction and length of the edge
        Vector3f edgeDirection = vertex2.sub(vertex1);
        float edgeLength = edgeDirection.length();
        edgeDirection.normalize();

        // Calculate the minimum and maximum values of t for intersection
        float tMin = 0.0f;
        float tMax = edgeLength;

        // Perform intersection tests with each axis of the AABB
        for (int i = 0; i < 3; i++) {
            if (Math.abs(edgeDirection.get(i)) < EPSILON) {
                // Edge is parallel to the AABB face, so no intersection possible
                if (vertex1.get(i) < aabb.getMin().get(i) || vertex1.get(i) > aabb.getMax().get(i)) {
                    return false;
                }
            } else {
                float t1 = (aabb.getMin().get(i) - vertex1.get(i)) / edgeDirection.get(i);
                float t2 = (aabb.getMax().get(i) - vertex1.get(i)) / edgeDirection.get(i);

                if (t1 > t2) {
                    float temp = t1;
                    t1 = t2;
                    t2 = temp;
                }

                if (t1 > tMin) {
                    tMin = t1;
                }

                if (t2 < tMax) {
                    tMax = t2;
                }

                if (tMin > tMax) {
                    return false;
                }
            }
        }

        // Check if the intersection occurred within the edge length
        return tMin <= edgeLength && tMax >= 0;
    }

    public static boolean isTriangleCollidingWithPlane(Triangle triangle, Plane plane) {

        List<Vector3f> vertices = triangle.getVertices();
        boolean allVerticesInFront = true;
        boolean allVerticesBehind = true;

        for (Vector3f vertex : vertices) {
            if (plane.isPointInFront(vertex)) {
                allVerticesBehind = false;
            } else {
                allVerticesInFront = false;
            }
        }

        return !(allVerticesInFront || allVerticesBehind);
    }

    public static boolean isCapsuleCollidingWithTriangle(Capsule capsule, Triangle triangle) {
        // Step 1: Check if any of the capsule's end points are inside the triangle.
        if (triangle.isPointInside(capsule.getStart()) || triangle.isPointInside(capsule.getEnd())) {
            return true;
        }

        // Step 2: Check if the capsule intersects the triangle's plane.
        Vector3f capsuleDirection = capsule.getEnd().sub(capsule.getStart());
        Vector3f triangleNormal = triangle.calculateTriangleNormal();
        float dotProduct = triangleNormal.dot(capsuleDirection);

        if (dotProduct == 0) {
            // Handle the parallel case as needed.
            return false;
        }

        Vector3f intersectionPoint = calculateIntersectionPoint(capsule, triangle, triangleNormal, dotProduct);

        // Step 3: Check if the intersection point is inside the triangle.
        if (triangle.isPointInside(intersectionPoint)) {
            return true;
        }

        // Step 4: Check for edge-edge collisions.
        Vector3f[] triangleVertices = {triangle.getVertex1(), triangle.getVertex2(), triangle.getVertex3()};
        Vector3f capsuleStart = capsule.getStart();
        Vector3f capsuleEnd = capsule.getEnd();

        for (int i = 0; i < 3; i++) {
            Vector3f triangleVertex1 = triangleVertices[i];
            Vector3f triangleVertex2 = triangleVertices[(i + 1) % 3];

            if (checkEdgeEdgeCollision(capsuleStart, capsuleEnd, triangleVertex1, triangleVertex2)) {
                return true;
            }
        }

        // If no collision conditions were met, return false.
        return false;
    }

    private static boolean checkEdgeEdgeCollision(Vector3f A, Vector3f B, Vector3f C, Vector3f D) {
        Vector3f AB = B.sub(A);
        Vector3f AC = C.sub(A);
        Vector3f AD = D.sub(A);

        Vector3f normalABCD = AB.cross(AC);

        // Check if the cross product of AB and AC points in the same direction as AD.
        if (normalABCD.dot(AD) > 0) {
            // Check if point D is on the opposite side of ABC as A is.
            Vector3f BC = C.sub(B);
            Vector3f BD = D.sub(B);
            Vector3f normalBCDA = BC.cross(BD);

            // The edges intersect.
            return normalBCDA.dot(AC) > 0 && normalBCDA.dot(AB) > 0;
        }

        return false;
    }

    private static Vector3f calculateIntersectionPoint(Capsule capsule, Triangle triangle, Vector3f triangleNormal, float dotProduct) {
        // Calculate the parameter along the capsule line where the intersection occurs.
        float t = triangleNormal.dot(triangle.getVertex1().sub(capsule.getStart())) / dotProduct;

        // Calculate the intersection point using the parameter.

        return capsule.getStart().add(capsule.getEnd().sub(capsule.getStart()).mul(t));
    }

    public static boolean isCapsuleCollidingWithOBB(Capsule capsule, OBB obb) {
        // Calculate the center and orientation of the OBB
        Vector3f obbCenter = obb.getCenter();
        List<Vector3f> obbAxes = obb.getAxis();
        Vector3f obbHalfExtents = obb.getHalfExtents();

        // Calculate the axis between the capsule's start and end points
        Vector3f capsuleAxis = capsule.getEnd().sub(capsule.getStart()).normalize();

        // Calculate the vector between the capsule's center and the OBB's center
        Vector3f centerDiff = obbCenter.sub(capsule.getStart());

        // Project the capsule axis onto the OBB axes
        float capsuleProjection = centerDiff.dot(capsuleAxis);
        float obbProjection;

        for (Vector3f obbAxis : obbAxes) {
            obbProjection = centerDiff.dot(obbAxis);
            float t = Math.max(-obbHalfExtents.x, Math.min(obbHalfExtents.x, obbProjection));
            float distance = centerDiff.sub(obbAxis.mul(t)).length();

            if (distance > capsule.getRadius() && capsuleProjection < 0) {
                // No overlap found on this axis, so shapes are not colliding
                return false;
            }
        }

        return true;
    }



}
