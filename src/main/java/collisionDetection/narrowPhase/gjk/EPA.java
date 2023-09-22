package collisionDetection.narrowPhase.gjk;

import collisionDetection.narrowPhase.collisionResult.CollisionResult;
import collisionDetection.util.CollisionUtil;
import math.Vector3f;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

import static math.Const.*;

public class EPA {

    public CollisionResult epaCollisionResult(GJKSupport shape1, GJKSupport shape2, Simplex simplex) {

        simplex.createFacesFromSimplex();

        List<Face> faces = new ArrayList<>(simplex.getFaces());

        for (int iteration = 0; iteration < GJK_EPA_MAX_ITERATORS; iteration++) {
            // Find the face in the list with the smallest penetration depth
            Face closestFace = findClosestFaceToOrigin(faces);

            // Calculate the support point in the direction of the face's normal
            Vector3f supportPoint = CollisionUtil.support(shape1, shape2, closestFace.getNormal());

            // Calculate the distance from the support point to the face
            float distance = closestFace.distanceToPoint(supportPoint);

            // If the distance is within a small tolerance, a collision has been found
            if (distance <= EPSILON) {
                // Extract collision information from the minimum penetration face
                return createResult(shape1, shape2, closestFace);
            }

            // Create new faces by connecting the support point to the edges of the minimum penetration face
            createNewFaces(faces, supportPoint, closestFace);

            // Remove the minimum penetration face from the list
            faces.remove(closestFace);
        }

        Face closestFace =faces.stream()
                .min(Comparator.comparing(Face::getDistanceToOrigin))
                .orElse(faces.get(0));// No collision
        return createResult(shape1, shape2, closestFace);

    }


    private CollisionResult createResult(GJKSupport shape1, GJKSupport shape2, Face closestFace) {
        Vector3f normal = closestFace.getNormal();
        float depth = calculatePenetrationDepth(closestFace);
        List<Vector3f> contactPoints = calculateContactPoints(shape1, shape2, closestFace);
        return new CollisionResult(true, normal, depth, contactPoints);
    }

    private float calculatePenetrationDepth(Face closestFace) {
        // You can calculate the penetration depth by using the dot product of
        // the origin (0,0,0) to any point on the plane (e.g., a vertex)
        // and the face's normal vector, then taking the absolute value.
        // The absolute value ensures that the penetration depth is always positive.
        Vector3f pointOnPlane = closestFace.getVertices()[0]; // Choose one of the vertices as a reference
        Vector3f originToPlane = pointOnPlane.negate(); // Assuming origin is at (0,0,0)
        return Math.abs(originToPlane.dot(closestFace.getNormal()));
    }

    private List<Vector3f> calculateContactPoints(GJKSupport shape1, GJKSupport shape2, Face face) {
        List<Vector3f> contactPoints = new ArrayList<>();


        // Calculate the barycentric coordinates of the origin on the face.
        float[] barycentricCoords = calculateBarycentricCoordinates(face);

        // Use the barycentric coordinates to find the contact point on the face.
        Vector3f contactPoint = interpolateVertices(face.getVertices(), barycentricCoords);

        // Add the contact point to the list.
        contactPoints.add(shape1.closestPoint(contactPoint));
        contactPoints.add(shape2.closestPoint(contactPoint));


        return contactPoints;
    }

    private float[] calculateBarycentricCoordinates(Face face) {
        // Calculate the barycentric coordinates of the origin on the face.
        // This involves projecting the origin onto the plane of the face and expressing
        // the point in terms of the face's vertices.

        Vector3f pointOnPlane = projectPointOntoPlane(face.getNormal(), face.getVertices()[0]);

        // Calculate the barycentric coordinates using the point on the plane and the face's vertices.
        float[] barycentricCoords = new float[3];

        for (int i = 0; i < 3; i++) {
            Vector3f edge = face.getVertices()[(i + 1) % 3].sub(face.getVertices()[i]);
            Vector3f pointToVertex = pointOnPlane.sub(face.getVertices()[i]);

            if (edge.lengthSquared() <= EPSILON) {
                barycentricCoords[i] = 0;
            } else {
                // Calculate the barycentric coordinate for each vertex.
                barycentricCoords[i] = pointToVertex.dot(edge) / edge.lengthSquared();
            }
        }

        return barycentricCoords;
    }

    private Vector3f interpolateVertices(Vector3f[] vertices, float[] weights) {
        // Interpolate the vertices of a triangle using barycentric coordinates.
        Vector3f result = new Vector3f(0, 0, 0);

        for (int i = 0; i < 3; i++) {
            result = result.add(vertices[i].mul(weights[i]));
        }

        return result;
    }

    private Vector3f projectPointOntoPlane(Vector3f planeNormal, Vector3f planePoint) {
        // Project a point onto a plane defined by its normal and a point on the plane.
        Vector3f pointToPlane = Vector3f.Zero.sub(planePoint);
        float distance = pointToPlane.dot(planeNormal);
        return Vector3f.Zero.sub(planeNormal.mul(distance));
    }

    private void createNewFaces(List<Face> faces, Vector3f supportPoint, Face minPenetrationFace) {
        // Iterate over the edges of the minimum penetration face
        int numEdges = minPenetrationFace.getVertices().length;
        for (int i = 0; i < numEdges; i++) {
            int j = (i + 1) % numEdges; // Next edge index
            Vector3f edgeStart = minPenetrationFace.getVertices()[i];
            Vector3f edgeEnd = minPenetrationFace.getVertices()[j];

            // Create a new face using the support point and the edge vertices
            Face newFace = new Face(new Vector3f[]{supportPoint, edgeStart, edgeEnd});

            // Add the new face to the list of faces
            faces.add(newFace);
        }
    }


    private Face findClosestFaceToOrigin(List<Face> faces) {

        Face minPenetrationFace = faces.get(0); // Initialize with the first face
        float minPenetrationDepth = minPenetrationFace.getDistanceToOrigin();

        for (Face face : faces) {
            float depth = face.getDistanceToOrigin();
            if (depth < minPenetrationDepth) {
                minPenetrationDepth = depth;
                minPenetrationFace = face;
            }
        }

        return minPenetrationFace;
    }


}
