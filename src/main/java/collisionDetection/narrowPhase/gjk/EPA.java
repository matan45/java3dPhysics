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
            Face minPenetrationFace = findFaceWithMinimumPenetration(faces);

            // Calculate the support point in the direction of the face's normal
            Vector3f supportPoint = CollisionUtil.support(shape1, shape2, minPenetrationFace.getNormal());

            // Calculate the distance from the support point to the face
            float distance = minPenetrationFace.distanceToPoint(supportPoint);

            // If the distance is within a small tolerance, a collision has been found
            if (distance < EPSILON) {
                // Extract collision information from the minimum penetration face
                return minPenetrationFace.extractCollisionResult();
            }

            // Create new faces by connecting the support point to the edges of the minimum penetration face
            createNewFaces(faces, supportPoint, minPenetrationFace);

            // Remove the minimum penetration face from the list
            faces.remove(minPenetrationFace);
        }

        return faces.stream()
                .min(Comparator.comparing(Face::getDistanceToOrigin))
                .orElse(faces.get(0)).extractCollisionResult();// No collision
    }

    private static void createNewFaces(List<Face> faces, Vector3f supportPoint, Face minPenetrationFace) {
        // Iterate over the edges of the minimum penetration face
        int numEdges = minPenetrationFace.getVertices().length;
        for (int i = 0; i < numEdges; i++) {
            int j = (i + 1) % numEdges; // Next edge index
            Vector3f edgeStart = minPenetrationFace.getVertices()[i];
            Vector3f edgeEnd = minPenetrationFace.getVertices()[j];

            // Create a new face using the support point and the edge vertices
            Face newFace = new Face(new Vector3f[]{edgeStart, edgeEnd, supportPoint});

            // Add the new face to the list of faces
            faces.add(newFace);
        }
    }


    private Face findFaceWithMinimumPenetration(List<Face> faces) {
        if (faces.isEmpty()) {
            throw new IllegalArgumentException("List of faces is empty.");
        }

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
