package collisionDetection.narrowPhase.sat;

import collisionDetection.narrowPhase.collisionResult.CollisionResult;
import collisionDetection.primitive.Line;
import math.Vector3f;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class SATSolver {

    public CollisionResult satCollisionResult(SATSupport shape1, SATSupport shape2, Set<Vector3f> allAxis) {

        // Calculate collision normal and penetration depth
        Vector3f normal = calculateCollisionNormal(shape1, shape2, allAxis);
        float depth = calculatePenetrationDepth(shape1, shape2, normal);

        // Calculate and add contact points
        List<Vector3f> contacts = calculateContactPoints(shape1, shape2, normal);

        // Create and return a CollisionResult
        return new CollisionResult(true, normal, depth, contacts);

    }

    public Vector3f calculateCollisionNormal(SATSupport shape1, SATSupport shape2, Set<Vector3f> allAxis) {
        Vector3f smallestOverlapAxis = new Vector3f();
        float smallestOverlap = Float.MAX_VALUE;

        for (Vector3f axis : allAxis) {
            float overlap = calculateOverlapOnAxis(shape1, shape2, axis);

            if (overlap < smallestOverlap) {
                smallestOverlap = overlap;
                smallestOverlapAxis = axis;
            }
        }


        return smallestOverlapAxis;
    }

    private float calculatePenetrationDepth(SATSupport shape1, SATSupport shape2, Vector3f normal) {
        // Calculate the projection of shapes onto the collision normal
        Interval projection1 = shape1.getInterval(normal);
        Interval projection2 = shape2.getInterval(normal);

        float A = projection1.getMin();
        float B = projection1.getMax();
        float C = projection2.getMin();
        float D = projection2.getMax();

        if (A <= C && B >= C)
            return Math.abs(C - B);
        return Math.abs(A - D);
    }

    private List<Vector3f> calculateContactPoints(SATSupport shape1, SATSupport shape2, Vector3f normal) {
        List<Vector3f> contactPoints = new ArrayList<>();

        // Assuming both shapes have vertices defining their geometry
        List<Vector3f> vertices1 = shape1.getVertices();
        List<Vector3f> vertices2 = shape2.getVertices();

        // Calculate the Minkowski Difference
        List<Vector3f> minkowskiDifference = new ArrayList<>();
        for (Vector3f vertex1 : vertices1) {
            for (Vector3f vertex2 : vertices2) {
                minkowskiDifference.add(vertex1.sub(vertex2));
            }
        }

        // Use the EPA algorithm to find the contact point(s)
        // EPA is an iterative algorithm that refines the contact point(s)
        // For simplicity, this is a simplified version and may not handle all edge cases
        // In practice, you should use a well-tested EPA implementation
        Vector3f contactPoint = new Vector3f();
        float minDepth = Float.MAX_VALUE;

        for (Vector3f supportDirection : minkowskiDifference) {
            Vector3f supportA = shape1.closestPoint(supportDirection);
            Vector3f supportB = shape2.closestPoint(supportDirection.negate()); // Negate for the opposite direction

            Vector3f support = supportA.sub(supportB);
            float depth = support.dot(normal);

            if (depth < minDepth) {
                minDepth = depth;
                contactPoint = support;
            }
        }

        // The contact point is the point on the Minkowski Difference closest to the origin
        if (contactPoint != null) {
            // Calculate the actual contact point on the shapes
            Vector3f actualContactPoint1 = shape1.closestPoint(contactPoint);
            Vector3f actualContactPoint2 = shape2.closestPoint(contactPoint); // Negate for the opposite direction

            contactPoints.add(actualContactPoint1);
            contactPoints.add(actualContactPoint2);
        }

        return contactPoints;
    }

    private Vector3f calculateEdgeIntersection(Line edge1, Line edge2) {
        Vector3f edge1Start = edge1.getStart();
        Vector3f edge1End = edge1.getEnd();
        Vector3f edge2Start = edge2.getStart();
        Vector3f edge2End = edge2.getEnd();

        Vector3f direction1 = edge1End.sub(edge1Start);
        Vector3f direction2 = edge2End.sub(edge2Start);

        Vector3f start1toStart2 = edge2Start.sub(edge1Start);

        Vector3f cross1 = direction1.cross(direction2);
        float sqrLengthCross1 = cross1.lengthSquared();

        if (sqrLengthCross1 == 0) {
            // The edges are parallel or colinear, no intersection.
            return null;
        }

        float t = start1toStart2.cross(direction2).lengthSquared() / sqrLengthCross1;
        float u = start1toStart2.cross(direction1).lengthSquared() / sqrLengthCross1;

        if (t >= 0.0f && t <= 1.0f && u >= 0.0f && u <= 1.0f) {
            // The edges intersect at a point within both line segments.
            return edge1Start.add(direction1.mul(t));
        }

        // The edges do not intersect within their line segments.
        return null;
    }

    private List<Line> findCollisionEdges(SATSupport shape, Vector3f normal) {
        List<Line> edges = new ArrayList<>();

        List<Vector3f> vertices = shape.getVertices();
        int numVertices = vertices.size();

        for (int i = 0; i < numVertices; i++) {
            Vector3f currentVertex = vertices.get(i);
            Vector3f nextVertex = vertices.get((i + 1) % numVertices);

            Vector3f edge = nextVertex.sub(currentVertex);

            // Check if the edge is parallel to the collision normal
            if (!edge.equals(normal) && !edge.equals(normal.negate())) {
                edges.add(new Line(currentVertex, nextVertex));
            }
        }

        return edges;
    }

    private float calculateOverlapOnAxis(SATSupport shape1, SATSupport shape2, Vector3f axis) {
        Interval interval1 = shape1.getInterval(axis);
        Interval interval2 = shape2.getInterval(axis);

        // Calculate the overlap between the projections.
        return Math.min(interval1.getMax(), interval2.getMax()) - Math.max(interval1.getMin(), interval2.getMin());
    }

}
