package collisionDetection.narrowPhase.rc;

import collisionDetection.narrowPhase.collisionResult.RayCastResult;
import collisionDetection.primitive.*;
import collisionDetection.util.CollisionUtil;
import math.Vector3f;

import java.util.List;

import static math.Const.EPSILON;

public class RayCast {

    private final RCSolver rcSolver;

    public RayCast() {
        this.rcSolver = new RCSolver();
    }

    public RayCastResult isCollide(Ray ray, Sphere sphere) {
        Vector3f oc = ray.getOrigin().sub(sphere.getCenter());
        float a = ray.getDirection().dot(ray.getDirection());
        float b = 2.0f * oc.dot(ray.getDirection());
        float c = oc.dot(oc) - sphere.getRadius() * sphere.getRadius();
        float discriminant = b * b - 4 * a * c;

        // Intersection
        return !(discriminant < 0) ? rcSolver.solve(ray, sphere) : new RayCastResult(); // No intersection
    }

    public RayCastResult isCollide(Ray ray, AABB aabb) {
        // Find the two intersection points on the AABB using the ray's closest point method
        Vector3f p1 = ray.closestPoint(aabb.getMin());
        Vector3f p2 = ray.closestPoint(aabb.getMax());

        // Check if the intersection points are within the AABB's boundaries
        boolean betweenX = (p1.x >= aabb.getMin().x && p1.x <= aabb.getMax().x) || (p2.x >= aabb.getMin().x && p2.x <= aabb.getMax().x);
        boolean betweenY = (p1.y >= aabb.getMin().y && p1.y <= aabb.getMax().y) || (p2.y >= aabb.getMin().y && p2.y <= aabb.getMax().y);
        boolean betweenZ = (p1.z >= aabb.getMin().z && p1.z <= aabb.getMax().z) || (p2.z >= aabb.getMin().z && p2.z <= aabb.getMax().z);

        // If there's an intersection along all axes, the AABB and ray collide
        return (betweenX && betweenY && betweenZ) ? rcSolver.solve(ray, aabb) : new RayCastResult();
    }

    public RayCastResult isCollide(Ray ray, Capsule capsule) {
        Vector3f rayOrigin = ray.getOrigin();
        Vector3f rayDirection = ray.getDirection();

        Vector3f capsuleStart = capsule.getStart();
        Vector3f capsuleEnd = capsule.getEnd();
        float capsuleRadius = capsule.getRadius();

        Vector3f ab = capsuleEnd.sub(capsuleStart);
        Vector3f ac = rayOrigin.sub(capsuleStart);

        float t = ac.dot(rayDirection);
        if (t < 0) t = 0;
        if (t > 1) t = 1;

        Vector3f h = rayDirection.mul(t).sub(ac);

        float a = ab.dot(ab);
        float b = 2 * h.dot(ab);
        float c = h.dot(h) - capsuleRadius * capsuleRadius;

        float discriminant = b * b - 4 * a * c;

        if (discriminant < 0)
            return new RayCastResult(); // No collision

        // Calculate the two potential intersection points
        float sqrtDiscriminant = (float) Math.sqrt(discriminant);
        float t1 = (-b - sqrtDiscriminant) / (2 * a);
        float t2 = (-b + sqrtDiscriminant) / (2 * a);

        return ((t1 >= 0 && t1 <= 1) || (t2 >= 0 && t2 <= 1)) ? rcSolver.solve(ray, capsule) : new RayCastResult();
    }

    public RayCastResult isCollide(Ray ray, Cylinder cylinder) {
        Vector3f rayOrigin = ray.getOrigin();
        Vector3f rayDirection = ray.getDirection();

        Vector3f cylinderCenter = cylinder.getCenter();
        float cylinderRadius = cylinder.getRadius();
        float cylinderHeight = cylinder.getHeight();

        Vector3f oc = rayOrigin.sub(cylinderCenter);

        float a = rayDirection.lengthSquared();
        float b = 2 * oc.dot(rayDirection);
        float c = oc.lengthSquared() - cylinderRadius * cylinderRadius;

        float discriminant = b * b - 4 * a * c;

        if (discriminant >= 0) {
            float sqrtDiscriminant = (float) Math.sqrt(discriminant);
            float t1 = (-b - sqrtDiscriminant) / (2 * a);
            float t2 = (-b + sqrtDiscriminant) / (2 * a);

            // Check if either intersection point is within the bounds of the cylinder's height
            return ((t1 >= 0 && t1 <= cylinderHeight) || (t2 >= 0 && t2 <= cylinderHeight)) ? rcSolver.solve(ray, cylinder) : new RayCastResult(); // Collision detected
        }

        return new RayCastResult(); // No collision
    }

    public RayCastResult isCollide(Ray ray, OBB obb) {
        Vector3f rayOrigin = ray.getOrigin();
        Vector3f rayDirection = ray.getDirection();

        Vector3f obbCenter = obb.getCenter();
        List<Vector3f> obbAxis = obb.getAxis();
        Vector3f size = obb.getHalfExtents();

        // Vector from ray origin to OBB center
        Vector3f p = obbCenter.sub(rayOrigin);

        // Calculate dot products between OBB axes and ray direction
        Vector3f f = new Vector3f(
                obbAxis.get(0).dot(rayDirection),
                obbAxis.get(1).dot(rayDirection),
                obbAxis.get(2).dot(rayDirection));

        // Calculate dot products between OBB axes and vector p
        Vector3f e = new Vector3f(
                obbAxis.get(0).dot(p),
                obbAxis.get(1).dot(p),
                obbAxis.get(2).dot(p));


        float[] t = new float[]{
                0, 0, 0, 0, 0, 0
        };
        // Calculate intersection intervals for each OBB axis
        for (int i = 0; i < 3; i++) {
            t[i * 2] = (e.get(i) + size.get(i)) / f.get(i); // min
            t[i * 2 + 1] = (e.get(i) - size.get(i)) / f.get(i); // max
        }

        // Calculate min and max for all intervals
        float min = Math.max(
                Math.max(
                        Math.min(t[0], t[1]),
                        Math.min(t[2], t[3])),
                Math.min(t[4], t[5])
        );
        float max = Math.min(
                Math.min(
                        Math.max(t[0], t[1]),
                        Math.max(t[2], t[3])),
                Math.max(t[4], t[5])
        );

        return (!(min > max) && !(max < 0)) ? rcSolver.solve(ray, obb) : new RayCastResult();// Collision detected
    }

    public RayCastResult isCollide(Ray ray, Plane plane) {
        Vector3f rayOrigin = ray.getOrigin();
        Vector3f rayDirection = ray.getDirection();

        Vector3f planeNormal = plane.getNormal();
        float planeDistance = plane.getDistance();

        float nd = rayDirection.dot(planeNormal);
        float pn = rayOrigin.dot(planeNormal);

        if (nd >= 0.0f)
            return new RayCastResult();

        float t = (planeDistance - pn) / nd;

        return (t >= 0) ? rcSolver.solve(ray, plane) : new RayCastResult(); // Collision detected
    }


    public RayCastResult isCollide(Ray ray, Triangle triangle) {
        Vector3f rayOrigin = ray.getOrigin();
        Vector3f rayDirection = ray.getDirection();

        Vector3f vertex1 = triangle.getVertex1();

        // Calculate the triangle's normal
        Vector3f edge1 = triangle.getEdge1();
        Vector3f edge2 = triangle.getEdge2();
        Vector3f triangleNormal = edge1.cross(edge2).normalize();

        // Calculate the denominator of the ray-plane intersection formula
        float nd = rayDirection.dot(triangleNormal);

        // Check if the ray is parallel or nearly parallel to the triangle
        if (Math.abs(nd) < EPSILON) {
            return new RayCastResult(); // No intersection
        }

        // Calculate the intersection point between the ray and the plane of the triangle
        float t = (vertex1.sub(rayOrigin)).dot(triangleNormal) / nd;

        // Check if the intersection point is outside the ray's range
        if (t < 0) {
            return new RayCastResult(); // Intersection behind the ray
        }

        // Calculate the point of intersection
        Vector3f intersectionPoint = rayOrigin.add(rayDirection.mul(t));

        // Calculate barycentric coordinates of the intersection point within the triangle
        Vector3f barycentricCords = CollisionUtil.barycentric(intersectionPoint, triangle);

        // Check if the intersection point is inside the triangle using barycentric coordinates
        return (barycentricCords.x >= 0 && barycentricCords.y >= 0 && barycentricCords.z >= 0) ? rcSolver.solve(ray, triangle) : new RayCastResult();
    }

    public RayCastResult isCollide(Ray ray, ConvexPolyhedron convexPolyhedron) {

        if (convexPolyhedron.isPointInside(ray.getOrigin()))
            return rcSolver.solve(ray, convexPolyhedron);

        // Find the closest point on the convex polyhedron to the ray's origin
        Vector3f closestPoint = convexPolyhedron.closestPoint(ray.getOrigin());

        // Calculate the direction from the ray's origin to the closest point
        Vector3f rayToClosestPoint = closestPoint.sub(ray.getOrigin());

        // If the closest point is outside the polyhedron, check if it's in the ray's direction
        // If it is, there is an intersection
        float dotProduct = rayToClosestPoint.dot(ray.getDirection());
        return (dotProduct >= 0) ? rcSolver.solve(ray, convexPolyhedron) : new RayCastResult();

    }

}
