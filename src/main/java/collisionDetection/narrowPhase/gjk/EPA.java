package collisionDetection.narrowPhase.gjk;

import collisionDetection.narrowPhase.collision_result.CollisionResult;
import math.Const;
import math.Vector3f;

import java.util.ArrayList;
import java.util.List;

public class EPA {
    private static final int MAX_ITERATORS = 50;
    private static final float MinimumContactSeparation = 1e-5f;

    public static CollisionResult epaCollisionResult(GJKSupport shape1, GJKSupport shape2, Simplex simplex) {
        if (simplex.containsOrigin()) {
            for (int iteration = 0; iteration < MAX_ITERATORS; iteration++) {
                // Find the closest face to the origin within the simplex.
                Face closestFace = simplex.findClosestFaceToOrigin();

                // Find the support point in the direction of the closest face's normal.
                Vector3f supportPoint = shape1.support(closestFace.getNormal())
                        .sub(shape2.support(closestFace.getNormal().negate()));

                float distanceToOrigin = closestFace.getNormal().dot(supportPoint);

                // Check if the distance to the origin has converged.
                if (Math.abs(distanceToOrigin - closestFace.getDistanceToOrigin()) < Const.EPSILON) {
                    // We have convergence; calculate the penetration depth and contact points.
                    Vector3f penetrationNormal = closestFace.getNormal().normalize();

                    // Find multiple contact points if needed
                    List<Vector3f> contacts = computeContactPoints(simplex, penetrationNormal);

                    return new CollisionResult(true, penetrationNormal, distanceToOrigin, contacts);
                }

                // Expand the simplex by adding the support point.
                simplex.addPoint(supportPoint);
            }
        }

        return new CollisionResult(false, new Vector3f(), 0, null);// No collision
    }

    private static List<Vector3f> computeContactPoints(Simplex simplex, Vector3f normal) {
        List<Vector3f> contacts = new ArrayList<>();

        // Determine the faces of the simplex involved in the collision.
        // You may need to adapt this part based on your specific simplex structure.
        Face[] collisionFaces = determineCollisionFaces(simplex, normal);

        for (Face face : collisionFaces) {
            // Compute contact point on the face.
            Vector3f contactPoint = computeContactPointOnFace(face, normal);

            // Check if the contact point is not too close to existing contact points.
            boolean isValidContact = true;
            for (Vector3f existingContact : contacts) {
                if (contactPoint.distance(existingContact) < MinimumContactSeparation) {
                    isValidContact = false;
                    break;
                }
            }

            if (isValidContact) {
                contacts.add(contactPoint);
            }
        }

        return contacts;
    }

    private static Face[] determineCollisionFaces(Simplex simplex, Vector3f normal) {
        List<Face> collisionFaces = new ArrayList<>();

        // Iterate through the faces of the simplex and check if each face is involved in the collision.
        for (int i = 0; i < simplex.getSize(); i++) {
            Face currentFace = simplex.getFace(i);

            // Compute the distance from the origin to the face.
            float distanceToFace = currentFace.getNormal().dot(new Vector3f(0, 0, 0));

            // Check if the face is facing the origin and is closer than a threshold.
            if (currentFace.getNormal().dot(normal) < -Const.EPSILON && distanceToFace < Const.EPSILON) {
                collisionFaces.add(currentFace);
            }
        }

        // Convert the list of collision faces to an array.
        return collisionFaces.toArray(new Face[0]);
    }

    private static Vector3f computeContactPointOnFace(Face face, Vector3f normal) {
        Vector3f[] vertices = face.getVertices();
        Vector3f origin = new Vector3f(0, 0, 0); // The origin point

        // Compute the vector from one vertex of the face to the origin
        Vector3f vertexToOrigin = origin.sub(vertices[0]);

        // Project the vector onto the face's normal to find the contact point
        float distance = vertexToOrigin.dot(normal);

        return origin.sub(normal.mul(distance));
    }
}
