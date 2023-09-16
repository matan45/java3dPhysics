package collisionDetection.narrowPhase.gjk;

import collisionDetection.narrowPhase.collisionResult.CollisionResult;
import collisionDetection.util.CollisionUtil;
import math.Vector3f;

import java.util.ArrayList;
import java.util.List;

import static math.Const.EPSILON;
import static math.Const.GJK_EPA_MAX_ITERATORS;

public class EPA {

    public CollisionResult epaCollisionResult(GJKSupport shape1, GJKSupport shape2, Simplex simplex) {

        simplex.createFacesFromSimplex();

        for (int iteration = 0; iteration < GJK_EPA_MAX_ITERATORS; iteration++) {
            // Find the closest face to the origin within the simplex.
            Face closestFace = findClosestFace(simplex);

            // Find the support point in the direction of the closest face normal
            Vector3f supportPoint = CollisionUtil.support(shape1, shape2, closestFace.getNormal());

            // Calculate the signed distance from the support point to the closest face
            float distance = closestFace.getNormal().dot(supportPoint);

            // If the distance is very close to zero, we have a collision
            if (Math.abs(distance) < EPSILON) {
                // Calculate collision normal (invert it if pointing inwards)
                Vector3f collisionNormal = closestFace.getNormal();
                if (distance < 0) {
                    collisionNormal.negate();
                }

                // Calculate penetration depth (absolute value of distance)
                float penetrationDepth = Math.abs(distance);

                // Calculate contact points by projecting the support point onto the face
                List<Vector3f> contactPoints = calculateContactPointsOnFace(closestFace, simplex);

                // Create and return the collision result
                return new CollisionResult(true, collisionNormal, penetrationDepth, contactPoints);
            }
            // Expand the simplex by adding the support point.
            simplex.pushFront(supportPoint);
        }


        return new CollisionResult();// No collision
    }

    public Face findClosestFace(Simplex simplex) {
        // Initialize variables to store the closest face and its distance
        Face closestFace = new Face();
        float closestDistance = Float.MAX_VALUE;

        // Iterate through the faces of the simplex
        for (Face currentFace : simplex.getFaces()) {

            // Calculate the signed distance from the origin to the face
            float distance = currentFace.getNormal().dot(simplex.getPoint(0)) - currentFace.getDistanceToOrigin();

            // Check if this face is closer than the closest one found so far
            if (distance < closestDistance) {
                closestDistance = distance;
                closestFace = currentFace;
            }
        }

        // Return the closest face
        return closestFace;
    }

    private List<Vector3f> calculateContactPointsOnFace(Face closestFace, Simplex simplex) {
        List<Vector3f> contactPoints = new ArrayList<>();

        // Iterate through the vertices of the simplex
        for (Vector3f vertex : simplex.getPoints()) {
            // Calculate the vector from the vertex to the face
            Vector3f vertexToFace = closestFace.getVertices()[0].sub(vertex);

            // Calculate the projection of the vertex onto the face
            float t = vertexToFace.dot(closestFace.getNormal()) / closestFace.getNormal().dot(closestFace.getNormal());
            Vector3f projectedPoint = vertex.add(closestFace.getNormal().mul(t));

            // Check if the projected point is inside the face
            if (isPointInsideFace(projectedPoint, closestFace)) {
                contactPoints.add(projectedPoint);
            }
        }

        return contactPoints;
    }

    private boolean isPointInsideFace(Vector3f point, Face face) {
        // Check if the point is inside the face by checking if it is on the correct side
        // of all the face's edges using the cross product.

        // Iterate through the edges of the face
        for (int i = 0; i < face.getVertices().length; i++) {
            Vector3f edgeStart = face.getVertices()[i];
            Vector3f edgeEnd = face.getVertices()[(i + 1) % face.getVertices().length];

            // Calculate the edge normal (perpendicular to the edge)
            Vector3f edgeNormal = edgeEnd.sub(edgeStart).cross(face.getNormal());

            // Calculate the vector from the edge start to the point
            Vector3f edgeToPoint = point.sub(edgeStart);

            // If the dot product of the edge normal and edge-to-point vector is positive,
            // the point is on the correct side of the edge.
            if (edgeNormal.dot(edgeToPoint) < 0) {
                return false; // Point is outside the face
            }
        }

        return true; // Point is inside the face
    }

}
