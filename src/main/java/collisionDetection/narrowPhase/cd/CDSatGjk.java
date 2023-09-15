package collisionDetection.narrowPhase.cd;

import collisionDetection.primitive.*;
import collisionDetection.primitive.terrain.TerrainShape;
import collisionDetection.util.CollisionUtil;
import math.Vector3f;

import java.util.ArrayList;
import java.util.List;

import static math.Const.EPSILON;

public class CDSatGjk {

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

        // Calculate the parameter t for the line equation (lineStart + t * lineDirection)
        float t = (planeDistance - lineStart.dot(planeNormal)) / dotProduct;

        // Check if the intersection point is within the line segment
        // Line intersects with the plane, but not within the line segment
        return t >= 0 && t <= 1; // Line intersects with the plane within the line segment

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

    public static boolean isCollide(Plane plane, ConvexPolyhedron convexPolyhedron) {
        // Iterate through the vertices of the convex polyhedron
        for (Vector3f vertex : convexPolyhedron.getVertices()) {
            // Check if the vertex is inside the plane
            if (plane.isPointInside(vertex)) {
                return true; // Collision detected
            }
        }

        // No vertices of the polyhedron are inside the plane, so there's no collision
        return false;
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
        Vector3f closestPointOnAxis = CollisionUtil.closestPointOnLineToAABB(aabb, new Line(capsule.getStart(), capsule.getEnd()));

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
        Vector3f triangleNormal = triangle.calculateFaceNormal();
        float triangleConstant = -triangleNormal.dot(triangle.getVertex1());

        // Check if cylinder cap collides with triangle plane
        float distanceToTopCap = CollisionUtil.calculateDistanceToPlane(cylinderTop, triangleNormal, triangleConstant);
        float distanceToBottomCap = CollisionUtil.calculateDistanceToPlane(cylinderBottom, triangleNormal, triangleConstant);

        if (Math.abs(distanceToTopCap) < cylinder.getRadius() || Math.abs(distanceToBottomCap) < cylinder.getRadius()) {
            return true;
        }

        // Check cylinder side collision
        return isCylinderSideCollidingWithTriangle(cylinder, triangle);
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
        boolean startColliding = isPointCollidingWithCylinderCap(capsule.getStart(), cylinder);
        boolean endColliding = isPointCollidingWithCylinderCap(capsule.getEnd(), cylinder);

        // Check if the capsule's body is colliding with the cylinder's body
        boolean bodyColliding = CollisionUtil.isLineCollidingWithCylinderBody(new Line(capsule.getStart(), capsule.getEnd()), cylinder);

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
        Vector3f cylinderAxis = cylinder.getUpAxis(); // Assuming the cylinder's axis is along the y-axis
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
        Vector3f triangleNormal = triangle.calculateFaceNormal();
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

            if (checkEdgeEdgeCollision(capsuleStart, capsuleEnd, triangleVertex1, triangleVertex2)) {
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


    public static boolean isCollide(Cylinder cylinder1, Cylinder cylinder2) {
        Vector3f center1 = cylinder1.getCenter();
        Vector3f center2 = cylinder2.getCenter();

        float distanceSq = center1.distanceSquared(center2);
        float sumOfRadii = cylinder1.getRadius() + cylinder2.getRadius();
        float sumOfHeights = cylinder1.getHeight() + cylinder2.getHeight();

        // Check for collision by comparing the squared distances
        return distanceSq <= (sumOfRadii * sumOfRadii) && distanceSq <= (sumOfHeights * sumOfHeights);
    }

    public static boolean isCollide(Sphere sphere, ConvexPolyhedron convexPolyhedron) {
        // Check if the sphere is inside the polyhedron
        if (convexPolyhedron.isPointInside(sphere.getCenter())) {
            return true;
        }

        // Check if any of the polyhedron's vertices are inside the sphere
        for (Vector3f vertex : convexPolyhedron.getVertices()) {
            if (sphere.isPointInside(vertex)) {
                return true;
            }
        }

        // Check for edge-sphere intersection
        for (int i = 0; i < convexPolyhedron.getVertices().size(); i++) {
            Vector3f v0 = convexPolyhedron.getVertices().get(i);
            Vector3f v1 = convexPolyhedron.getVertices().get((i + 1) % convexPolyhedron.getVertices().size());
            Vector3f edge = v1.sub(v0);

            // Check if the closest point on the edge is inside the sphere
            if (sphere.isPointInside(edge)) {
                return true;
            }
        }

        // If none of the above conditions are met, there is no collision
        return false;
    }

    public static boolean isCollide(Capsule capsule, ConvexPolyhedron convexPolyhedron) {
        // Check if any point on the capsule is inside the convex polyhedron
        Vector3f start = capsule.getStart();
        Vector3f end = capsule.getEnd();

        if (convexPolyhedron.isPointInside(start) || convexPolyhedron.isPointInside(end)) {
            return true;
        }

        // Check if any point on the convex polyhedron is inside the capsule
        List<Vector3f> vertices = convexPolyhedron.getVertices();
        for (Vector3f vertex : vertices) {
            if (capsule.isPointInside(vertex)) {
                return true;
            }
        }

        // Check for edge-edge intersection (not implemented here)
        return isEdgeEdgeIntersection(capsule, convexPolyhedron);
    }

    private static boolean isEdgeEdgeIntersection(Capsule capsule, ConvexPolyhedron convexPolyhedron) {
        List<Vector3f> capsuleVertices = new ArrayList<>();
        capsuleVertices.add(capsule.getStart());
        capsuleVertices.add(capsule.getEnd());

        List<Vector3f> polyhedronVertices = convexPolyhedron.getVertices();
        List<Vector3f> polyhedronEdges = new ArrayList<>();

        // Create a list of edges for the polyhedron
        for (int i = 0; i < polyhedronVertices.size(); i++) {
            Vector3f v1 = polyhedronVertices.get(i);
            Vector3f v2 = polyhedronVertices.get((i + 1) % polyhedronVertices.size());
            polyhedronEdges.add(v1);
            polyhedronEdges.add(v2);
        }

        // Iterate over each edge of the capsule
        for (int i = 0; i < capsuleVertices.size(); i++) {
            Vector3f capsuleStart = capsuleVertices.get(i);
            Vector3f capsuleEnd = capsuleVertices.get((i + 1) % capsuleVertices.size());

            // Iterate over each edge of the polyhedron
            for (int j = 0; j < polyhedronEdges.size(); j += 2) {
                Vector3f polyhedronEdgeStart = polyhedronEdges.get(j);
                Vector3f polyhedronEdgeEnd = polyhedronEdges.get(j + 1);

                // Check for intersection between the two line segments
                if (doLineSegmentsIntersect(capsuleStart, capsuleEnd, polyhedronEdgeStart, polyhedronEdgeEnd)) {
                    return true;
                }
            }
        }

        return false;
    }

    public static boolean doLineSegmentsIntersect(Vector3f p1, Vector3f q1, Vector3f p2, Vector3f q2) {
        Vector3f e1 = q1.sub(p1);
        Vector3f e2 = q2.sub(p2);
        Vector3f h = e2.cross(e1);
        float f = h.dot(e1);

        // Check if the line segments are parallel (no intersection)
        if (f > -1e-6f && f < 1e-6f) {
            return false;
        }

        float s = h.dot(p1.sub(p2)) / f;
        if (s < 0.0f || s > 1.0f) {
            return false;
        }

        float t = h.dot(p2.sub(p1)) / f;
        return !(t < 0.0f) && !(t > 1.0f);
    }


    public static boolean isCollide(Cylinder cylinder, ConvexPolyhedron convexPolyhedron) {
        // Check if the cylinder's closest point to the polyhedron is inside the polyhedron
        Vector3f closestPointToPolyhedron = convexPolyhedron.closestPoint(cylinder.getCenter());
        if (!convexPolyhedron.isPointInside(closestPointToPolyhedron)) {
            return false; // The closest point is outside the polyhedron, no collision.
        }

        // Check if the closest point on the cylinder to the polyhedron is inside the cylinder
        return cylinder.isPointInside(closestPointToPolyhedron);
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

    public static boolean isCollide(Line line, TerrainShape terrainShape) {
        return terrainShape.isCollide(line);
    }

    private static boolean isCylinderSideCollidingWithTriangle(Cylinder cylinder, Triangle triangle) {
        // Calculate the triangle's normal
        Vector3f triangleNormal = triangle.calculateFaceNormal();

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
        float distanceSquared = point.sub(circleCenter).lengthSquared();
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

    private static boolean isPointCollidingWithCylinderCap(Vector3f point, Cylinder cylinder) {
        // Calculate the distance between the point and the cylinder's center in the XZ plane
        float distanceXZ = (float) Math.sqrt((point.x - cylinder.getCenter().x) * (point.x - cylinder.getCenter().x) +
                (point.z - cylinder.getCenter().z) * (point.z - cylinder.getCenter().z));

        // Check if the point is within the cylinder's cap radius and its Y coordinate is within the cap's height
        return distanceXZ <= cylinder.getRadius() && Math.abs(point.y - cylinder.getCenter().y) <= cylinder.getHeight() / 2.0f;
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

}
