package collisionDetection.narrowPhase.rc;

import collisionDetection.narrowPhase.collisionResult.RayCastResult;
import collisionDetection.primitive.*;
import math.Vector3f;

import java.util.List;

import static math.Const.EPSILON;

public class RCSolver {

    public static RayCastResult solve(Ray ray, Sphere sphere) {
        // Calculate the vector from the ray's origin to the sphere's center
        Vector3f rayToSphere = sphere.getCenter().sub(ray.getOrigin());

        // Calculate the dot product of the ray direction and the vector to the sphere's center
        float tDot = rayToSphere.dot(ray.getDirection());

        // Calculate the discriminant to determine if there's an intersection
        float discriminant = tDot * tDot - rayToSphere.dot(rayToSphere) + sphere.getRadius() * sphere.getRadius();

        // Calculate the two possible solutions for t
        float t1 = -tDot + (float) Math.sqrt(discriminant);
        float t2 = -tDot - (float) Math.sqrt(discriminant);

        // Find the closest intersection point along the ray
        float t = Math.min(t1, t2);

        // Calculate the intersection point and surface normal
        Vector3f intersectionPoint = ray.getOrigin().add(ray.getDirection().mul(t));
        Vector3f surfaceNormal = intersectionPoint.sub(sphere.getCenter()).normalize();

        // Create a RayCastResult object with the intersection information
        return new RayCastResult(intersectionPoint, surfaceNormal, t, true);
    }

    public static RayCastResult solve(Ray ray, AABB aabb) {
        // Calculate the parameter values for each pair of min and max planes
        float tMinX = (aabb.getMin().x - ray.getOrigin().x) / ray.getDirection().x;
        float tMaxX = (aabb.getMax().x - ray.getOrigin().x) / ray.getDirection().x;
        float tMinY = (aabb.getMin().y - ray.getOrigin().y) / ray.getDirection().y;
        float tMaxY = (aabb.getMax().y - ray.getOrigin().y) / ray.getDirection().y;
        float tMinZ = (aabb.getMin().z - ray.getOrigin().z) / ray.getDirection().z;
        float tMaxZ = (aabb.getMax().z - ray.getOrigin().z) / ray.getDirection().z;

        // Calculate the intervals along the ray for the intersections with each plane
        float tEnter = Math.max(
                Math.max(Math.min(tMinX, tMaxX), Math.min(tMinY, tMaxY)),
                Math.min(tMinZ, tMaxZ)
        );

        // Calculate the intersection point and normal
        Vector3f intersectionPoint = ray.getOrigin().add(ray.getDirection().mul(tEnter));

        // Calculate the normal by finding which plane was hit
        Vector3f normal = new Vector3f(0.0f, 0.0f, 0.0f);
        if (tEnter == tMinX) normal.x = -1.0f;
        else if (tEnter == tMaxX) normal.x = 1.0f;
        else if (tEnter == tMinY) normal.y = -1.0f;
        else if (tEnter == tMaxY) normal.y = 1.0f;
        else if (tEnter == tMinZ) normal.z = -1.0f;
        else normal.z = 1.0f;

        // Create a RayCastResult object with the intersection information
        return new RayCastResult(intersectionPoint, normal.normalize(), tEnter, true);

    }

    public static RayCastResult solve(Ray ray, Capsule capsule) {
        // Calculate the capsule axis vector and vectors to the start and end points
        Vector3f axis = capsule.getEnd().sub(capsule.getStart());
        Vector3f vecToRayOrigin = ray.getOrigin().sub(capsule.getStart());

        // Calculate projection of vecToRayOrigin onto the capsule axis
        float t = vecToRayOrigin.dot(axis) / axis.dot(axis);

        // Calculate the closest point on the capsule axis to the ray origin
        t = Math.max(0, Math.min(1, t)); // Clamp t to [0, 1]
        Vector3f closestPointOnAxis = capsule.getStart().add(axis.mul(t));

        // Calculate the closest point on the capsule's surface to the ray origin
        Vector3f vecToClosestPoint = ray.getOrigin().sub(closestPointOnAxis);
        float distanceToAxisSquared = vecToClosestPoint.dot(vecToClosestPoint);

        if (t <= 0) {
            // Closest point is at the start cap of the capsule
            return checkIntersectionWithCap(ray, capsule.getStart());
        } else if (t >= 1) {
            // Closest point is at the end cap of the capsule
            return checkIntersectionWithCap(ray, capsule.getEnd());
        }

        // The ray intersects with the capsule's axis
        // Calculate the intersection point and normal
        Vector3f intersectionPoint = closestPointOnAxis.add(
                ray.getDirection().mul(
                        (float) Math.sqrt(capsule.getRadius() * capsule.getRadius() - distanceToAxisSquared)
                )
        );
        Vector3f normal = intersectionPoint.sub(closestPointOnAxis).normalize();

        return new RayCastResult(intersectionPoint, normal, t, true);
    }

    public static RayCastResult solve(Ray ray, Cylinder cylinder) {
        // Calculate the direction vector from the ray's origin to the cylinder's center
        Vector3f rayToCenter = cylinder.getCenter().sub(ray.getOrigin());

        // Project the rayToCenter vector onto the cylinder's upAxis to get the distance along the cylinder's axis
        float tAxis = rayToCenter.dot(cylinder.getUpAxis());

        // Calculate the point on the ray's direction vector that is closest to the cylinder's axis
        Vector3f closestPointOnRay = ray.getOrigin().add(ray.getDirection().mul(tAxis));

        // Calculate the vector from the closest point on the ray to the cylinder's center
        Vector3f vectorToCenter = cylinder.getCenter().sub(closestPointOnRay);

        // Calculate the intersection point and normal
        Vector3f intersectionPoint = closestPointOnRay.add(vectorToCenter.normalize().mul(cylinder.getRadius()));
        Vector3f normal = intersectionPoint.sub(cylinder.getCenter()).normalize();

        return new RayCastResult(intersectionPoint, normal, tAxis, true);

    }

    public static RayCastResult solve(Ray ray, OBB obb) {
        List<Vector3f> obbAxis = obb.getAxis();
        // Transform the ray's origin and direction into the OBB's local space
        Vector3f localOrigin = ray.getOrigin().sub(obb.getCenter());
        Vector3f localDirection = new Vector3f(
                localOrigin.dot(obbAxis.get(0)),
                localOrigin.dot(obbAxis.get(1)),
                localOrigin.dot(obbAxis.get(2))
        );

        // Initialize values for the ray's entry and exit along the OBB's slabs
        float tMin = Float.NEGATIVE_INFINITY;
        float tMax = Float.POSITIVE_INFINITY;

        // Perform slab-based intersection tests for each axis of the OBB
        for (int i = 0; i < 3; i++) {
            float axisDotDirection = localDirection.dot(obbAxis.get(i));
            float axisDotOrigin = localOrigin.dot(obbAxis.get(i));

            float t1 = (-obb.getHalfExtents().get(i) - axisDotOrigin) / axisDotDirection;
            float t2 = (obb.getHalfExtents().get(i) - axisDotOrigin) / axisDotDirection;

            // Ensure t1 is less than or equal to t2
            if (t1 > t2) {
                float temp = t1;
                t1 = t2;
                t2 = temp;
            }

            // Update tMin and tMax
            tMin = Math.max(tMin, t1);
            tMax = Math.min(tMax, t2);

        }

        // Calculate the intersection point and normal in the OBB's local space
        Vector3f localIntersection = localOrigin.add(localDirection.mul(tMin));
        Vector3f localNormal = new Vector3f();

        for (int i = 0; i < 3; i++) {
            localNormal = localNormal.add(obbAxis.get(i).mul(Math.abs(localIntersection.get(i)) - obb.getHalfExtents().get(i)));
        }

        // Transform the intersection point and normal back to world space
        Vector3f intersectionPoint = obb.getCenter().add(localIntersection);
        Vector3f normal = new Vector3f(
                localNormal.dot(obbAxis.get(0)),
                localNormal.dot(obbAxis.get(1)),
                localNormal.dot(obbAxis.get(2))
        ).normalize();

        return new RayCastResult(intersectionPoint, normal, tMin, true);
    }


    public static RayCastResult solve(Ray ray, Plane plane) {
        // Calculate the dot product of the ray direction and the plane's normal
        float dotDirectionNormal = ray.getDirection().dot(plane.getNormal());

        // Calculate the parameter t for the intersection point
        float t = (plane.getDistance() - ray.getOrigin().dot(plane.getNormal())) / dotDirectionNormal;

        // Calculate the intersection point
        Vector3f intersectionPoint = ray.getOrigin().add(ray.getDirection().mul(t));

        // The normal of the plane is already known
        Vector3f normal = plane.getNormal();

        return new RayCastResult(intersectionPoint, normal, t, true);
    }

    public static RayCastResult solve(Ray ray, Triangle triangle) {
        // Compute the triangle's normal
        Vector3f normal = triangle.getEdge1().cross(triangle.getEdge2()).normalize();

        // Compute the denominator of the ray-plane intersection formula
        float denominator = normal.dot(ray.getDirection());

        // Calculate the parameter t for the intersection point
        float t = (triangle.getVertex1().sub(ray.getOrigin()).dot(normal)) / denominator;

        // Calculate the intersection point on the triangle's plane
        Vector3f intersectionPoint = ray.getOrigin().add(ray.getDirection().mul(t));

        // The intersection point is inside the triangle
        return new RayCastResult(intersectionPoint, normal, t, true);

    }

    public static RayCastResult solve(Ray ray, ConvexPolyhedron convexPolyhedron) {
        Vector3f closestPoint = new Vector3f();
        float minDistance = Float.POSITIVE_INFINITY;

        for (int i = 0; i < convexPolyhedron.getVertices().size(); i++) {
            Vector3f v0 = convexPolyhedron.getVertices().get(i);
            Vector3f v1 = convexPolyhedron.getVertices().get((i + 1) % convexPolyhedron.getVertices().size());

            // Calculate the normal of the plane defined by the edge and the ray direction
            Vector3f edge = v1.sub(v0);
            Vector3f edgeNormal = edge.cross(ray.getDirection()).normalize();

            // Check if the edge is nearly parallel to the ray direction
            float dotEdgeDirection = Math.abs(edgeNormal.dot(ray.getDirection()));
            if (dotEdgeDirection < EPSILON) {
                continue; // Skip nearly parallel edges
            }

            // Calculate the parameter t for the intersection with the plane defined by the edge
            float t = edgeNormal.dot(v0.sub(ray.getOrigin())) / dotEdgeDirection;

            // Check if t is negative or greater than the current minimum distance
            if (t < 0 || t > minDistance) {
                continue; // No intersection or farther than the current closest point
            }

            // Calculate the intersection point on the plane defined by the edge
            Vector3f intersectionPoint = ray.getOrigin().add(ray.getDirection().mul(t));

            // Check if the intersection point is inside the edge segment
            float edgeLength = edge.length();
            float dotEdge = intersectionPoint.sub(v0).dot(edge);
            if (dotEdge >= 0 && dotEdge <= edgeLength * edgeLength) {
                // Update the closest point and minimum distance
                float distance = intersectionPoint.distance(ray.getOrigin());
                if (distance < minDistance) {
                    closestPoint = intersectionPoint;
                    minDistance = distance;
                }
            }
        }

        // Calculate the normal at the closest point (using the closest edge's normal)
        Vector3f edgeNormal = closestPoint.sub(ray.getOrigin()).normalize();
        return new RayCastResult(closestPoint, edgeNormal, minDistance, true);
    }


    private static RayCastResult checkIntersectionWithCap(Ray ray, Vector3f capCenter) {
        // Calculate the intersection point on the cap
        Vector3f vecToCapCenter = ray.getOrigin().sub(capCenter);
        float t = vecToCapCenter.dot(ray.getDirection());

        if (t < 0) {
            // The ray starts inside the cap
            t = 0;
        }

        Vector3f intersectionPoint = ray.getOrigin().add(ray.getDirection().mul(t));

        // Calculate the normal pointing outward from the cap
        Vector3f normal = intersectionPoint.sub(capCenter).normalize();

        return new RayCastResult(intersectionPoint, normal, t, true);

    }


}
