package collisionDetection.narrowPhase;

import collisionDetection.primitive.*;
import math.Vector3f;

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

            if (Line.isLineColliding(line, polyhedronEdge)) {
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
            if (Line.isLineColliding(line, edge)) {
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

}
