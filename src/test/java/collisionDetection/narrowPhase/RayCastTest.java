package collisionDetection.narrowPhase;

import collisionDetection.narrowPhase.rc.RayCast;
import collisionDetection.primitive.*;
import math.Vector3f;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class RayCastTest {

    @Test
    public void testRayIntersectsSphere() {
        Ray ray = new Ray(new Vector3f(0, 0, 0), new Vector3f(1, 0, 0)); // Example ray
        Sphere sphere = new Sphere(new Vector3f(2, 0, 0), 1.0f); // Example sphere

        assertTrue(RayCast.isCollide(ray, sphere));
    }

    @Test
    public void testRayMissesSphere() {
        Ray ray = new Ray(new Vector3f(0, 0, 0), new Vector3f(0, 1, 0)); // Example ray
        Sphere sphere = new Sphere(new Vector3f(3, 0, 0), 1.0f); // Example sphere

        assertFalse(RayCast.isCollide(ray, sphere));
    }


    @Test
    public void testRayIntersectsABB() {
        AABB aabb = new AABB(new Vector3f(0, 0, 0), new Vector3f(2, 2, 2));
        Ray ray = new Ray(new Vector3f(1, 1, 5), new Vector3f(0, 0, -1));

        // Test for collision
        assertTrue(RayCast.isCollide(ray, aabb));
    }

    @Test
    public void testRayMissesAABB() {
        AABB aabb = new AABB(new Vector3f(0, 0, 0), new Vector3f(2, 2, 2));
        Ray ray = new Ray(new Vector3f(1, 1, 5), new Vector3f(0, -1, 0));

        // Test for no collision
        assertFalse(RayCast.isCollide(ray, aabb));
    }

    @Test
    public void testRayIntersectsCapsule() {
        Capsule capsule = new Capsule(new Vector3f(0, 0, 0), new Vector3f(0, 4, 0), 1.0f);
        Ray ray = new Ray(new Vector3f(0, 5, 0), new Vector3f(0, -1, 0));

        // Test for collision
        assertTrue(RayCast.isCollide(ray, capsule));
    }

    @Test
    public void testRayMissesCapsule() {
        Capsule capsule = new Capsule(new Vector3f(0, 0, 0), new Vector3f(0, 4, 0), 1.0f);
        Ray ray = new Ray(new Vector3f(2, 2, 2), new Vector3f(0, 0, -1));

        // Test for no collision
        assertFalse(RayCast.isCollide(ray, capsule));
    }


    @Test
    public void testRayIntersectsCylinder() {
        Vector3f rayOrigin = new Vector3f(0, 0, 0);
        Vector3f rayDirection = new Vector3f(1, 0, 0);
        Ray ray = new Ray(rayOrigin, rayDirection);

        Vector3f cylinderCenter = new Vector3f(1, 0, 0);
        float cylinderRadius = 0.5f;
        float cylinderHeight = 2.0f;
        Cylinder cylinder = new Cylinder(cylinderCenter, cylinderRadius, cylinderHeight);

        assertTrue(RayCast.isCollide(ray, cylinder));
    }

    @Test
    public void testRayMissesCylinder() {
        Vector3f rayOrigin = new Vector3f(0, 0, 0);
        Vector3f rayDirection = new Vector3f(1, 0, 0);
        Ray ray = new Ray(rayOrigin, rayDirection);

        Vector3f cylinderCenter = new Vector3f(3, 0, 0); // Move cylinder out of the ray's path
        float cylinderRadius = 0.5f;
        float cylinderHeight = 2.0f;
        Cylinder cylinder = new Cylinder(cylinderCenter, cylinderRadius, cylinderHeight);

        boolean collision = RayCast.isCollide(ray, cylinder);
        assertFalse(collision);
    }


    @Test
    public void testRayIntersectsPlane() {
        Vector3f rayOrigin = new Vector3f(0, 2, 0);
        Vector3f rayDirection = new Vector3f(0, -1, 0);
        Ray ray = new Ray(rayOrigin, rayDirection);

        Vector3f planeNormal = new Vector3f(0, 1, 0);
        float planeDistance = 1;
        Plane plane = new Plane(planeNormal, planeDistance);

        boolean collision = RayCast.isCollide(ray, plane);
        assertTrue(collision);
    }

    @Test
    public void testRayMissesPlane() {
        Vector3f rayOrigin = new Vector3f(0, 0, 0);
        Vector3f rayDirection = new Vector3f(1, 0, 0);
        Ray ray = new Ray(rayOrigin, rayDirection);

        Vector3f planeNormal = new Vector3f(0, 1, 0);
        float planeDistance = 2;
        Plane plane = new Plane(planeNormal, planeDistance);

        boolean collision = RayCast.isCollide(ray, plane);
        assertFalse(collision);
    }

    @Test
    public void testRayIntersectsOBB() {
        OBB obb = new OBB(new Vector3f(0, 0, 0), new Vector3f(1, 1, 1));
        Ray ray = new Ray(new Vector3f(0, -1, 0), new Vector3f(0, 1, 0));

        // Test for collision
        assertTrue(RayCast.isCollide(ray, obb));
    }

    @Test
    public void testRayMissesOBB() {
        OBB obb = new OBB(new Vector3f(0, 0, 0), new Vector3f(2, 2, 2));
        Ray ray = new Ray(new Vector3f(0, 3, 0), new Vector3f(0, 1, 0));

        // Test for no collision
        assertFalse(RayCast.isCollide(ray, obb));
    }

    @Test
    void testRayIntersectsTriangle() {
        Triangle triangle = new Triangle(new Vector3f(0, 0, 1), new Vector3f(1, 1, 1), new Vector3f(1, 0, 1));
        Ray ray = new Ray(new Vector3f(0, 0, 0), new Vector3f(1, 1, 1));

        assertTrue(RayCast.isCollide(ray, triangle));
    }

    @Test
    void testRayMissTriangle() {
        Triangle triangle = new Triangle(new Vector3f(0, 0, 0), new Vector3f(1, 0, 0), new Vector3f(0, 1, 0));
        Ray ray = new Ray(new Vector3f(2, 2, 2), new Vector3f(0, 0, -1));

        assertFalse(RayCast.isCollide(ray, triangle));
    }

    @Test
    public void testRayIntersectsConvexPolyhedron() {
        // Create a convex polyhedron with vertices (0,0,0), (1,0,0), (0,1,0)
        // This is a simple triangle in the XY plane.
        ConvexPolyhedron convexPolyhedron = new ConvexPolyhedron(List.of(
                new Vector3f(0, 0, 0),
                new Vector3f(1, 0, 0),
                new Vector3f(0, 1, 0),
                new Vector3f(1, 1, 0)
        ));

        // Create a ray originating from (0.5, 0.5, 1) in the positive Z direction.
        Ray ray = new Ray(new Vector3f(0.5f, 0.5f, 0), new Vector3f(0, 1, 0));

        // Check if the ray intersects the convex polyhedron
        boolean result = RayCast.isCollide(ray, convexPolyhedron);

        // Assert that the ray intersects the polyhedron
        assertTrue(result);

        // Create a ray originating from (0.5, 0.5, 1) in the positive Z direction.
        Ray ray2 = new Ray(new Vector3f(-2, 0, 0), new Vector3f(1, 0, 0));

        // Check if the ray intersects the convex polyhedron
        boolean result2 = RayCast.isCollide(ray2, convexPolyhedron);

        // Assert that the ray intersects the polyhedron
        assertTrue(result2);
    }



    @Test
    public void testRayMissesConvexPolyhedron() {
        // Create a convex polyhedron with vertices (0,0,0), (1,0,0), (0,1,0)
        // This is a simple triangle in the XY plane.
        ConvexPolyhedron convexPolyhedron = new ConvexPolyhedron(List.of(
                new Vector3f(0, 0, 0),
                new Vector3f(1, 0, 0),
                new Vector3f(0, 1, 0)
        ));

        Ray ray = new Ray(new Vector3f(0, 8, 0), new Vector3f(0, 1, 0));

        // Check if the ray intersects the convex polyhedron
        boolean result = RayCast.isCollide(ray, convexPolyhedron);

        // Assert that the ray does not intersect the polyhedron
        assertFalse(result);
    }


}