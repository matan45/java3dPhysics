package collisionDetection.narrowPhase;

import collisionDetection.narrowPhase.sat.Interval;
import collisionDetection.primitive.*;
import collisionDetection.util.CollisionUtil;
import math.Vector3f;

import java.util.List;

import static math.Const.EPSILON;

public class CollisionDetection {
    public static boolean isCollide(Line line, AABB aabb) {
        // Check if any of the line's endpoints are inside the AABB.
        if (aabb.isPointInside(line.getStart()) || aabb.isPointInside(line.getEnd())) {
            return true;
        }

        // Check if any of the AABB's corner points are inside the line.
        Vector3f[] aabbCorners = new Vector3f[]{
                new Vector3f(aabb.getMin().x, aabb.getMin().y, aabb.getMin().z),
                new Vector3f(aabb.getMin().x, aabb.getMin().y, aabb.getMax().z),
                new Vector3f(aabb.getMin().x, aabb.getMax().y, aabb.getMin().z),
                new Vector3f(aabb.getMin().x, aabb.getMax().y, aabb.getMax().z),
                new Vector3f(aabb.getMax().x, aabb.getMin().y, aabb.getMin().z),
                new Vector3f(aabb.getMax().x, aabb.getMin().y, aabb.getMax().z),
                new Vector3f(aabb.getMax().x, aabb.getMax().y, aabb.getMin().z),
                new Vector3f(aabb.getMax().x, aabb.getMax().y, aabb.getMax().z)
        };

        for (Vector3f corner : aabbCorners) {
            if (line.isPointInside(corner)) {
                return true;
            }
        }

        // Check if the line intersects any of the AABB's faces.
        // You can do this by checking if the closest point on the line to the center of the AABB
        // is inside the AABB, and if it is, check if it lies between the endpoints of the line.
        Vector3f closestPointOnLine = line.closestPoint(aabb.getCenter());
        if (aabb.isPointInside(closestPointOnLine)) {
            return closestPointOnLine.isBetween(line.getStart(), line.getEnd());
        }

        return false;
    }

    public static boolean isCollide(Line line, OBB obb) {
        // First, find the closest point on the line to the OBB
        Vector3f closestPointOnLine = line.closestPoint(obb.getCenter());

        // Next, check if the closest point on the line is inside the OBB
        return obb.isPointInside(closestPointOnLine);
    }

    public static boolean isCollide(Line line, ConvexPolyhedron convexPolyhedron) {
        // Check if any of the polyhedron's vertices are inside the line.
        for (Vector3f vertex : convexPolyhedron.getVertices()) {
            if (line.isPointInside(vertex)) {
                return true;
            }
        }

        // Check if any of the line's endpoints are inside the polyhedron.
        if (convexPolyhedron.isPointInside(line.getStart()) || convexPolyhedron.isPointInside(line.getEnd())) {
            return true;
        }

        // Check if the line intersects any of the polyhedron's edges.
        for (int i = 0; i < convexPolyhedron.getVertices().size(); i++) {
            Vector3f v0 = convexPolyhedron.getVertices().get(i);
            Vector3f v1 = convexPolyhedron.getVertices().get((i + 1) % convexPolyhedron.getVertices().size());
            Line polyhedronEdge = new Line(v0, v1);

            if (isCollide(line, polyhedronEdge)) {
                return true;
            }
        }

        return false;
    }

    public static boolean isCollide(Line line, Capsule capsule) {
        // First, calculate the closest point on the line to the capsule's axis.
        Vector3f axis = capsule.getEnd().sub(capsule.getStart());
        Vector3f vecToLineStart = line.getStart().sub(capsule.getStart());

        float t = vecToLineStart.dot(axis) / axis.dot(axis);
        t = Math.max(0, Math.min(1, t)); // Clamp t to [0, 1]

        Vector3f closestPointOnAxis = capsule.getStart().add(axis.mul(t));

        // Then, calculate the closest point on the line to the closest point on the capsule's axis.
        Vector3f closestPointOnLine = line.closestPoint(closestPointOnAxis);

        // Check if the closest point on the line to the capsule's axis is within the capsule's radius.
        return closestPointOnLine.distanceSquared(closestPointOnAxis) <= capsule.getRadius() * capsule.getRadius();
    }

    public static boolean isCollide(Line line, Cylinder cylinder) {
        // Calculate the direction vector of the line segment.
        Vector3f dir = line.getEnd().sub(line.getStart());

        // Calculate the vector from the start of the line to the cylinder's center.
        Vector3f toCylinder = cylinder.getCenter().sub(line.getStart());

        // Calculate the dot product of the direction vector and the line-to-cylinder vector.
        float t = dir.dot(toCylinder) / dir.lengthSquared();

        // Ensure t is within the valid range [0, 1] for a point on the line segment.
        t = Math.max(0, Math.min(1, t));

        // Calculate the closest point on the line to the cylinder's center.
        Vector3f closestPointOnLine = line.getStart().add(dir.mul(t));

        // Check if the closest point on the line is inside the cylinder.
        return cylinder.isPointInside(closestPointOnLine);
    }

    public static boolean isCollide(Line line, Sphere sphere) {
        // Calculate the closest point on the line to the sphere's center.
        Vector3f closestPoint = line.closestPoint(sphere.getCenter());

        // Check if the closest point is inside the sphere and lies within the line segment.
        return sphere.isPointInside(closestPoint) && line.isPointInside(closestPoint);
    }

    public static boolean isCollide(Line line, Triangle triangle) {
        // First, check if any of the line's endpoints are inside the triangle.
        if (triangle.isPointInside(line.getStart()) || triangle.isPointInside(line.getEnd())) {
            return true;
        }

        // Next, check if any of the triangle's vertices are inside the line segment.
        if (line.isPointInside(triangle.getVertex1()) ||
                line.isPointInside(triangle.getVertex2()) ||
                line.isPointInside(triangle.getVertex3())) {
            return true;
        }

        // Finally, check for intersection between the line and the triangle's edges.
        Vector3f[] triangleVertices = new Vector3f[]{
                triangle.getVertex1(), triangle.getVertex2(), triangle.getVertex3()
        };

        Line[] triangleEdges = new Line[]{
                new Line(triangleVertices[0], triangleVertices[1]),
                new Line(triangleVertices[1], triangleVertices[2]),
                new Line(triangleVertices[2], triangleVertices[0])
        };

        for (Line edge : triangleEdges) {
            if (isCollide(line, edge)) {
                return true;
            }
        }

        return false;
    }


    public static boolean isCollide(Line line, Plane plane) {
        // Calculate the direction vector of the line
        Vector3f lineDirection = line.getEnd().sub(line.getStart());

        // Check if the line is parallel to the plane (dot product of their directions is close to 0)
        float dotProduct = lineDirection.dot(plane.getNormal());
        if (Math.abs(dotProduct) < EPSILON) {
            return false; // Line is parallel to the plane and does not intersect
        }

        // Calculate the point of intersection between the line and the plane
        Vector3f lineStart = line.getStart();
        Vector3f planeNormal = plane.getNormal();
        float planeDistance = plane.getDistance();

        // Calculate the parameter 't' for the line equation (lineStart + t * lineDirection)
        float t = (planeDistance - lineStart.dot(planeNormal)) / dotProduct;

        // Check if the intersection point is within the line segment
        // Line intersects with the plane, but not within the line segment
        return t >= 0 && t <= 1; // Line intersects with the plane within the line segment

    }

    public static boolean isCollide(Line line, TerrainShape terrainShape) {
        return terrainShape.isCollide(line);
    }

    public static boolean isCollide(Sphere sphere, AABB aabb) {
        Vector3f closestPoint = aabb.closestPoint(sphere.getCenter());
        float distSq = sphere.getCenter().sub(closestPoint).lengthSquared();
        float radiusSq = sphere.getRadius() * sphere.getRadius();
        return distSq < radiusSq;
    }

    public static boolean isCollide(Sphere sphere, Plane plane) {
        Vector3f closestPoint = plane.closestPoint(sphere.getCenter());
        float distSq = sphere.getCenter().sub(closestPoint).lengthSquared();
        float radiusSq = sphere.getRadius() * sphere.getRadius();
        return distSq < radiusSq;
    }

    public static boolean isCollide(Sphere sphere, Triangle triangle) {
        Vector3f closestPoint = triangle.closestPoint(sphere.getCenter());
        float distSq = sphere.getCenter().sub(closestPoint).lengthSquared();
        float radiusSq = sphere.getRadius() * sphere.getRadius();
        return distSq < radiusSq;
    }

    public static boolean isCollide(Sphere sphere, OBB obb) {
        Vector3f closestPoint = obb.closestPoint(sphere.getCenter());
        float distSq = sphere.getCenter().sub(closestPoint).lengthSquared();
        float radiusSq = sphere.getRadius() * sphere.getRadius();
        return distSq < radiusSq;
    }

    public static boolean isCollide(AABB aabb, OBB obb) {
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

    public static boolean isCollide(Sphere sphere, Capsule capsule) {
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

    public static boolean isCollide(Sphere sphere, Cylinder cylinder) {
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

    public static boolean isCollide(Plane plane, AABB aabb) {
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


    public static boolean isCollide(Capsule capsule, AABB aabb) {
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
        Vector3f closestPointOnAxis = CollisionUtil.closestPointOnSegmentToAABB(aabb, new Line(capsule.getStart(), capsule.getEnd()));

        // Calculate the squared distance from the closest point on the axis to the AABB
        float distanceToAxisSquared = aabb.closestPoint(closestPointOnAxis).lengthSquared();

        // If the squared distance to the axis is greater than the radius squared,
        // the capsule is not colliding with the AABB
        return distanceToAxisSquared <= radiusSquared;
    }

    public static boolean isCollide(Cylinder cylinder, AABB aabb) {
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

    public static boolean isCollide(Cylinder cylinder, Triangle triangle) {
        Vector3f cylinderTop = cylinder.getCenter().add(new Vector3f(0, cylinder.getHeight() / 2, 0));
        Vector3f cylinderBottom = cylinder.getCenter().add(new Vector3f(0, cylinder.getHeight() / 2, 0));

        // Calculate triangle's normal and constant term for the plane equation
        Vector3f triangleNormal = triangle.calculateTriangleNormal();
        float triangleConstant = -triangleNormal.dot(triangle.getVertex1());

        // Check if cylinder cap collides with triangle plane
        float distanceToTopCap = CollisionUtil.calculateDistanceToPlane(cylinderTop, triangleNormal, triangleConstant);
        float distanceToBottomCap = CollisionUtil.calculateDistanceToPlane(cylinderBottom, triangleNormal, triangleConstant);

        if (Math.abs(distanceToTopCap) < cylinder.getRadius() || Math.abs(distanceToBottomCap) < cylinder.getRadius()) {
            return true;
        }

        // Check cylinder side collision
        return CollisionUtil.isCylinderSideCollidingWithTriangle(cylinder, triangle);
    }



    public static boolean isCollide(Cylinder cylinder, Plane plane) {
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

    public static boolean isCollide(Capsule capsule, Plane plane) {
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

    public static boolean isCollide(Capsule capsule, Cylinder cylinder) {
        // Check if either of the capsule's ends is colliding with the cylinder's caps
        boolean startColliding = CollisionUtil.isPointCollidingWithCylinderCap(capsule.getStart(), cylinder);
        boolean endColliding = CollisionUtil.isPointCollidingWithCylinderCap(capsule.getEnd(), cylinder);

        // Check if the capsule's body is colliding with the cylinder's body
        boolean bodyColliding = CollisionUtil.isSegmentCollidingWithCylinderBody(new Line(capsule.getStart(), capsule.getEnd()),cylinder);

        // If any of the capsule's ends are colliding with the cylinder's caps or the capsule's body
        // is colliding with the cylinder's body, they are colliding
        return startColliding || endColliding || bodyColliding;
    }

    public static boolean isCollide(Cylinder cylinder, OBB obb) {
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
        return CollisionUtil.cylinderOBBCapsFacesIntersect(cylinder, obb); // No collision
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

    public static boolean isCollide(Triangle triangle, AABB aabb) {

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

    public static boolean isCollide(Triangle triangle, Plane plane) {

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

    public static boolean isCollide(Capsule capsule, Triangle triangle) {
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

        Vector3f intersectionPoint = CollisionUtil.calculateIntersectionPoint(capsule, triangle, triangleNormal, dotProduct);

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

            if (CollisionUtil.checkEdgeEdgeCollision(capsuleStart, capsuleEnd, triangleVertex1, triangleVertex2)) {
                return true;
            }
        }

        // If no collision conditions were met, return false.
        return false;
    }

    public static boolean isCollide(Capsule capsule, OBB obb) {
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

    public static boolean isCollide(Capsule capsule1, Capsule capsule2) {
        float radiusSum = capsule1.getRadius() + capsule2.getRadius();

        // Calculate the squared distance between the capsules' start points
        float distanceSquared = capsule1.getStart().distanceSquared(capsule2.getStart());

        // Check if the distance is less than the sum of the radii
        if (distanceSquared <= (radiusSum * radiusSum)) {
            return true;
        }

        // Calculate the squared distance between the capsules' start points and the end points of each capsule
        float distanceSquaredToC1Start = capsule1.getStart().distanceSquared(capsule2.getEnd());
        float distanceSquaredToC1End = capsule1.getEnd().distanceSquared(capsule2.getStart());
        float distanceSquaredToC2Start = capsule1.getStart().distanceSquared(capsule2.getEnd());
        float distanceSquaredToC2End = capsule1.getEnd().distanceSquared(capsule2.getEnd());

        // Check if any of the distances are less than the sum of the radii
        return distanceSquaredToC1Start <= (radiusSum * radiusSum) ||
                distanceSquaredToC1End <= (radiusSum * radiusSum) ||
                distanceSquaredToC2Start <= (radiusSum * radiusSum) ||
                distanceSquaredToC2End <= (radiusSum * radiusSum);
    }

    public static boolean isCollide(ConvexPolyhedron convexPolyhedron1, ConvexPolyhedron convexPolyhedron2) {
        List<Vector3f> vertices1 = convexPolyhedron1.getVertices();
        List<Vector3f> vertices2 = convexPolyhedron2.getVertices();

        // Loop through all the edges of both polyhedra
        for (int i = 0; i < vertices1.size(); i++) {
            Vector3f edgeStart = vertices1.get(i);
            Vector3f edgeEnd = vertices1.get((i + 1) % vertices1.size());

            Vector3f axis = edgeEnd.sub(edgeStart).normalize();

            if (isAxisSeparating(axis, convexPolyhedron1, convexPolyhedron2)) {
                return false;
            }
        }

        for (int i = 0; i < vertices2.size(); i++) {
            Vector3f edgeStart = vertices2.get(i);
            Vector3f edgeEnd = vertices2.get((i + 1) % vertices2.size());

            Vector3f axis = edgeEnd.sub(edgeStart).normalize();

            if (isAxisSeparating(axis, convexPolyhedron1, convexPolyhedron2)) {
                return false;
            }
        }

        return true;
    }

    private static boolean isAxisSeparating(Vector3f axis, ConvexPolyhedron convexPolyhedron1, ConvexPolyhedron convexPolyhedron2) {
        // Project the OBBs onto the axis
        Interval projection1 = convexPolyhedron1.getInterval(axis);
        Interval projection2 = convexPolyhedron2.getInterval(axis);

        // Check for separation between the intervals
        return projection1.getMax() < projection2.getMin() || projection2.getMax() < projection1.getMin();
    }

    public static boolean isCollide(Line line1, Line line2) {
        // Get the direction vectors of both lines.
        Vector3f dir1 = line1.getEnd().sub(line1.getStart());
        Vector3f dir2 = line2.getEnd().sub(line2.getStart());

        // Calculate the determinant of the direction vectors.
        float determinant = dir1.x * dir2.y - dir1.y * dir2.x;

        // If the determinant is close to zero, the lines are parallel and may not intersect.
        if (Math.abs(determinant) < EPSILON) {
            return false;
        }

        // Calculate parameters for the lines' parametric equations.
        Vector3f toStart2 = line2.getStart().sub(line1.getStart());
        float t1 = (toStart2.x * dir2.y - toStart2.y * dir2.x) / determinant;
        float t2 = (toStart2.x * dir1.y - toStart2.y * dir1.x) / determinant;

        // Check if the intersection points are within the valid range [0, 1] for both lines.
        return t1 >= 0 && t1 <= 1 && t2 >= 0 && t2 <= 1;  // Lines intersect within their segments.
    }

    public static boolean isCollide(Cylinder cylinder1, Cylinder cylinder2) {
        Vector3f center1 = cylinder1.getCenter();
        Vector3f center2 = cylinder2.getCenter();

        float distanceSq = center1.distanceSquared(center2);
        float sumOfRadii = cylinder1.getRadius() + cylinder2.getRadius();
        float sumOfHeights = cylinder1.getHeight() + cylinder2.getHeight();

        // Check for collision by comparing the squared distances
        return distanceSq <= (sumOfRadii * sumOfRadii) && distanceSq <= (sumOfHeights * sumOfHeights);
    }

    public static boolean isCollide(OBB obb1, OBB obb2) {
        // Combine axes from both OBBs
        Vector3f[] test = new Vector3f[18];

        test[0] = obb1.getAxis().get(0);
        test[1] = obb1.getAxis().get(1);
        test[2] = obb1.getAxis().get(2);
        test[3] = obb2.getAxis().get(0);
        test[4] = obb2.getAxis().get(1);
        test[5] = obb2.getAxis().get(2);

        for (int i = 0; i < 3; ++i) {
            test[6 + i * 3] = test[i].cross(test[3]);
            test[6 + i * 3 + 1] = test[i].cross(test[4]);
            test[6 + i * 3 + 2] = test[i].cross(test[5]);
        }

        // Include edge normals of both OBBs
        for (int i = 0; i < 3; ++i) {
            test[12 + i] = obb1.getEdge(i).normalize();
            test[15 + i] = obb2.getEdge(i).normalize();
        }

        // Check for separation along each axis
        for (Vector3f axis : test) {
            if (isAxisSeparating(axis, obb1, obb2)) {
                return false; // No collision along this axis
            }
        }

        return true; // No separation along any axis, collision detected
    }

    private static boolean isAxisSeparating(Vector3f axis, OBB obb1, OBB obb2) {
        // Project the OBBs onto the axis
        Interval projection1 = obb1.getInterval(axis);
        Interval projection2 = obb2.getInterval(axis);

        // Check for separation between the intervals
        return projection1.getMax() < projection2.getMin() || projection2.getMax() < projection1.getMin();
    }

    public static boolean isCollide(Plane plane1, Plane plane2) {
        Vector3f normal1 = plane1.getNormal();
        Vector3f normal2 = plane2.getNormal();

        // Check if the normals are parallel (i.e., planes are colliding)
        return normal1.normalize().equals(normal2.normalize());
    }

    public static boolean isCollide(Sphere sphere1, Sphere sphere2) {
        float distanceSquared = sphere1.getCenter().distanceSquared(sphere2.getCenter());
        float radiusSum = sphere1.getRadius() + sphere2.getRadius();

        return distanceSquared <= (radiusSum * radiusSum);
    }

    public static boolean isCollide(Triangle triangle1, Triangle triangle2) {
        // Axes to test
        Vector3f[] axes = {
                triangle1.calculateTriangleNormal(),
                triangle2.calculateTriangleNormal(),
                triangle1.getEdge1().normalize(),
                triangle1.getEdge2().normalize(),
                triangle1.getEdge3().normalize(),
                triangle2.getEdge1().normalize(),
                triangle2.getEdge2().normalize(),
                triangle2.getEdge3().normalize()
        };

        for (Vector3f axis : axes) {
            if (isSeparatingAxis(axis, triangle1, triangle2)) {
                return false; // No collision along this axis
            }
        }

        return true; // No separation along any axis, collision detected
    }

    private static boolean isSeparatingAxis(Vector3f axis, Triangle triangle1, Triangle triangle2) {
        Interval interval1 = triangle1.getInterval(axis);
        Interval interval2 = triangle2.getInterval(axis);

        return interval1.getMax() < interval2.getMin() || interval2.getMax() < interval1.getMin();
    }

}
