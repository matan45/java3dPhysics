package collisionDetection.util.hull;

import math.Vector3f;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class QuickHull3D {
    private final List<Vector3f> points;
    private final List<Vector3f> convexHullVertices;
    public final List<Face> convexHullFaces; // This list holds the faces of the convex hull

    public QuickHull3D(List<Vector3f> points) {
        this.points = points;
        convexHullVertices = new ArrayList<>();
        convexHullFaces = new ArrayList<>();
    }

    public List<Vector3f> computeConvexHull() {
        if (points.size() < 4) {
            return points; // Convex hull is the points themselves
        }

        // Find the initial tetrahedron to start the algorithm
        Vector3f[] initialTetrahedron = findInitialTetrahedron(points);
        convexHullVertices.addAll(Arrays.asList(initialTetrahedron));

        // Find the point furthest from the initial tetrahedron
        Vector3f furthestPoint = findFurthestPoint(initialTetrahedron, points);
        // Recursively build the convex hull starting with the furthest point
        buildConvexHull(furthestPoint, convexHullVertices);

        return convexHullVertices;
    }

    private Vector3f[] findInitialTetrahedron(List<Vector3f> points) {
        // Find the point with the minimum x-coordinate
        Vector3f minX = points.get(0);
        for (Vector3f point : points) {
            if (point.x < minX.x) {
                minX = point;
            }
        }

        // Find the point with the maximum x-coordinate
        Vector3f maxX = points.get(0);
        for (Vector3f point : points) {
            if (point.x > maxX.x) {
                maxX = point;
            }
        }

        // Find the point with the minimum y-coordinate
        Vector3f minY = points.get(0);
        for (Vector3f point : points) {
            if (point.y < minY.y) {
                minY = point;
            }
        }

        // Find the point with the maximum y-coordinate
        Vector3f maxY = points.get(0);
        for (Vector3f point : points) {
            if (point.y > maxY.y) {
                maxY = point;
            }
        }

        // Find the point with the minimum z-coordinate
        Vector3f minZ = points.get(0);
        for (Vector3f point : points) {
            if (point.z < minZ.z) {
                minZ = point;
            }
        }

        // Find the point with the maximum z-coordinate
        Vector3f maxZ = points.get(0);
        for (Vector3f point : points) {
            if (point.z > maxZ.z) {
                maxZ = point;
            }
        }

        // Create the initial tetrahedron using the found points
        Vector3f[] initialTetrahedron = new Vector3f[4];
        initialTetrahedron[0] = minX;
        initialTetrahedron[1] = maxX;
        initialTetrahedron[2] = minY;
        initialTetrahedron[3] = maxY;

        return initialTetrahedron;
    }

    private void buildConvexHull(Vector3f furthestPoint, List<Vector3f> faceVertices) {
        if (points.isEmpty()) {
            return;
        }
        // Create new faces and divide the points into visible and non-visible sets
        List<Face> newFaces = createNewFaces(furthestPoint, faceVertices);
        List<Vector3f> visiblePoints = separatePointsByVisibleFaces(faceVertices);

        // Recursively build the convex hull for the visible points
        if (!visiblePoints.isEmpty()) {
            Vector3f newFurthestPoint = findFurthestPoint(faceVertices.toArray(new Vector3f[0]), visiblePoints);
            buildConvexHull(newFurthestPoint, visiblePoints);
        }

        // Update convexHullVertices with the newly added vertices
        convexHullVertices.addAll(visiblePoints);

        // Combine the faces and update adjacency information
        combineFaces(newFaces);
    }

    private List<Vector3f> separatePointsByVisibleFaces(List<Vector3f> points) {
        List<Vector3f> visiblePoints = new ArrayList<>();
        for (Vector3f point : points) {
            boolean isVisible = false;
            for (Face face : convexHullFaces) {
                if (calculateDistanceToFace(point, face.getVertices()) > 0) {
                    isVisible = true;
                    break;
                }
            }

            if (isVisible) {
                visiblePoints.add(point);
            }
        }
        return visiblePoints;
    }

    private void combineFaces(List<Face> newFaces) {
        for (Face newFace : newFaces) {
            List<Face> visibleFaces = new ArrayList<>();
            for (Face existingFace : convexHullFaces) {
                if (isVisibleFromFace(newFace, existingFace)) {
                    visibleFaces.add(existingFace);
                }
            }

            updateAdjacency(newFace, visibleFaces);

            convexHullFaces.removeAll(visibleFaces);
            convexHullFaces.add(newFace);
        }
    }

    private void updateAdjacency(Face newFace, List<Face> visibleFaces) {
        for (Face visibleFace : visibleFaces) {
            newFace.addAdjacentFace(visibleFace);
            visibleFace.addAdjacentFace(newFace);
        }
    }

    private boolean isVisibleFromFace(Face testFace, Face existingFace) {
        Vector3f existingNormal = existingFace.getNormal();

        Vector3f centroidVector = calculateCentroidVector(testFace, existingFace);

        float dotProduct = centroidVector.dot(existingNormal);

        return dotProduct > 0;
    }

    private Vector3f calculateCentroidVector(Face face1, Face face2) {
        Vector3f centroid1 = calculateCentroid(face1);
        Vector3f centroid2 = calculateCentroid(face2);

        return centroid2.sub(centroid1);
    }

    private Vector3f calculateCentroid(Face face) {
        Vector3f sum = new Vector3f();

        for (Vector3f vertex : face.getVertices()) {
            sum.add(vertex);
        }

        int numVertices = face.getVertices().length;
        return sum.div(numVertices);
    }


    private List<Face> createNewFaces(Vector3f furthestPoint, List<Vector3f> visibleVertices) {
        List<Face> newFaces = new ArrayList<>();

        for (int i = 0; i < visibleVertices.size(); i++) {
            for (int j = i + 1; j < visibleVertices.size(); j++) {

                Vector3f[] vertices = new Vector3f[3];
                vertices[0] = visibleVertices.get(i);
                vertices[1] = visibleVertices.get(j);
                vertices[2] = furthestPoint;

                // Optionally calculate normal and orientation of the new face
                Vector3f normal = calculateNormalFromVertices(vertices);

                Face newFace = new Face(vertices, normal);

                newFaces.add(newFace);
            }
        }
        return newFaces;
    }


    private Vector3f findFurthestPoint(Vector3f[] vertices, List<Vector3f> points) {
        double maxDistance = 0;
        Vector3f furthestPoint = null;

        for (Vector3f point : points) {
            double distance = calculateDistanceToFace(point, vertices);
            if (distance > maxDistance) {
                maxDistance = distance;
                furthestPoint = point;
            }
        }

        return furthestPoint;
    }

    private double calculateDistanceToFace(Vector3f point, Vector3f[] faceVertices) {
        // Calculate the normal vector of the face using its vertices
        Vector3f faceNormal = calculateNormalFromVertices(faceVertices);

        // Calculate the vector from any vertex of the face to the point
        Vector3f vertexToPoint = new Vector3f(
                point.x - faceVertices[0].x,
                point.y - faceVertices[0].y,
                point.z - faceVertices[0].z
        );

        // Calculate the dot product between the vertex-to-point vector and the face normal
        double dotProduct = vertexToPoint.dot(faceNormal);

        // Calculate the projected point on the face by subtracting the projection offset
        double projectedX = point.x - dotProduct * faceNormal.x;
        double projectedY = point.y - dotProduct * faceNormal.y;
        double projectedZ = point.z - dotProduct * faceNormal.z;

        // Calculate the distance between the projected point and the original point
        return Math.sqrt(
                Math.pow(projectedX - point.x, 2) +
                        Math.pow(projectedY - point.y, 2) +
                        Math.pow(projectedZ - point.z, 2)
        );
    }

    private Vector3f calculateNormalFromVertices(Vector3f[] vertices) {
        // Calculate two vectors on the plane of the face
        Vector3f vector1 = new Vector3f(
                vertices[1].x - vertices[0].x,
                vertices[1].y - vertices[0].y,
                vertices[1].z - vertices[0].z
        );

        Vector3f vector2 = new Vector3f(
                vertices[2].x - vertices[0].x,
                vertices[2].y - vertices[0].y,
                vertices[2].z - vertices[0].z
        );

        // Calculate the cross product of the two vectors to get the normal vector
        Vector3f normal = vector1.cross(vector2);

        return normal.normalize(); // Normalize the normal vector;
    }

}
