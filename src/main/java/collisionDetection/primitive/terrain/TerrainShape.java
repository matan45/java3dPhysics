package collisionDetection.primitive.terrain;

import collisionDetection.narrowPhase.CollisionDetection;
import collisionDetection.narrowPhase.rc.RayCast;
import collisionDetection.primitive.AABB;
import collisionDetection.primitive.Line;
import collisionDetection.primitive.Ray;
import collisionDetection.primitive.Triangle;
import math.Vector2f;
import math.Vector3f;

import java.util.ArrayList;
import java.util.List;

import static math.Const.EPSILON;

public class TerrainShape {

    private float[][] heightData;
    private final List<TerrainTriangle> triangles;
    private AABB borders;
    private Vector3f terrainScale;
    private Vector3f terrainCenter;

    public TerrainShape(float[][] heightData, AABB borders, Vector3f terrainScale) {
        this.heightData = heightData;
        this.borders = borders;
        this.terrainCenter = borders.getCenter();
        this.triangles = new ArrayList<>();
        //by default is need to be 1,1,1
        if (terrainScale.equals(Vector3f.Zero))
            this.terrainScale = new Vector3f(1, 1, 1);
        else
            this.terrainScale = terrainScale;

        computeTriangles();
    }

    public AABB getBorders() {
        return borders;
    }

    public void setHeightData(float[][] heightData) {
        this.heightData = heightData;
        computeTriangles();
    }

    public void setBorders(AABB borders) {
        this.borders = borders;
        this.terrainCenter = borders.getCenter();
    }

    public void setTerrainScale(Vector3f terrainScale) {
        this.terrainScale = terrainScale;
    }

    private float getHeightOfTerrain(float worldX, float worldZ) {
        Vector2f position = getGridPosition(worldX, worldZ);
        return getHeightInterpolated(position.x, position.y);
    }

    private Vector2f getGridPosition(float worldX, float worldZ) {
        float positionX = Math.abs(worldX - terrainCenter.x) / terrainScale.x;
        float positionZ = Math.abs(worldZ - terrainCenter.z) / terrainScale.z;
        return new Vector2f(positionX, positionZ);
    }

    private float getHeightInterpolated(float x, float z) {
        float X0Z0Height = heightData[(int) x][(int) z];

        if (((int) x + 1 >= heightData.length) || ((int) z + 1 >= heightData[0].length)) {
            return X0Z0Height;
        }

        float X1Z0Height = heightData[(int) x + 1][(int) z];
        float X0Z1Height = heightData[(int) x][(int) z + 1];
        float X1Z1Height = heightData[(int) x + 1][(int) z + 1];

        float factorX = (float) (x - Math.floor(x));

        float interpolatedBottom = (X1Z0Height - X0Z0Height) * factorX + X0Z0Height;
        float interpolatedTop = (X1Z1Height - X0Z1Height) * factorX + X0Z1Height;

        float factorZ = (float) (z - Math.floor(z));

        float finalHeight = (interpolatedTop - interpolatedBottom) * factorZ + interpolatedBottom;


        return finalHeight * terrainScale.y;
    }

    public boolean isPointOnGround(Vector3f position) {
        if (isNotPointInside(position))
            return false;
        float groundPosition = getHeightOfTerrain(position.x, position.y);
        return position.y - groundPosition < EPSILON;
    }

    public boolean isPointBlowGround(Vector3f position) {
        if (isNotPointInside(position))
            return false;
        float groundPosition = getHeightOfTerrain(position.x, position.y);
        return position.y < groundPosition;
    }

    public Vector3f rayTerrainIntersection(Ray ray) {
        // Check if the ray intersects with the terrain's bounding box (borders)
        if (!RayCast.isCollide(ray, borders)) {
            return null; // Ray doesn't intersect with the terrain's bounding box
        }
        Vector2f position = getGridPosition(ray.getOrigin().x, ray.getOrigin().z);
        Ray rayForTest = new Ray(new Vector3f(position.x, ray.getOrigin().y, position.y),
                ray.getDirection());
        // Initialize variables to store the intersection point and distance
        Vector3f intersectionPoint = new Vector3f();
        float minDistance = Float.MAX_VALUE;

        // Iterate through the terrain grid and check for intersections
        for (TerrainTriangle triangle : triangles) {
            // Calculate the intersection point with the current quad
            Vector3f quadIntersection = rayQuadIntersection(rayForTest, triangle.toTriangle());

            // If there is an intersection, and it is closer than previous intersections, update the result
            if (quadIntersection != null) {
                float distance = ray.getOrigin().distance(quadIntersection);
                if (distance < minDistance) {
                    minDistance = distance;
                    intersectionPoint.set(quadIntersection);
                }
            }
        }

        // If there was an intersection, return the intersection point; otherwise, return null
        if (minDistance != Float.MAX_VALUE) {
            return intersectionPoint;
        }

        return null; // No collision
    }

    private Vector3f rayQuadIntersection(Ray ray, Triangle triangle) {

        Vector3f edge1 = triangle.getEdge1();
        Vector3f edge2 = triangle.getEdge2();

        Vector3f h = ray.getDirection().cross(edge2);
        float a = edge1.dot(h);

        if (a > -EPSILON && a < EPSILON) {
            return null; // Ray is parallel to the quad; no intersection.
        }

        float f = 1.0f / a;
        Vector3f s = ray.getOrigin().sub(triangle.getVertex1());
        float u = f * s.dot(h);

        if (u < 0.0f || u > 1.0f) {
            return null; // Intersection is outside the quad.
        }

        Vector3f q = s.cross(edge1);
        float v = f * ray.getDirection().dot(q);

        if (v < 0.0f || u + v > 1.0f) {
            return null; // Intersection is outside the quad.
        }

        // At this point, we have an intersection.
        float t = f * edge2.dot(q);
        if (t > EPSILON) {
            return ray.getOrigin().add(ray.getDirection().mul(t));
        }
        return null; // Intersection point is behind the ray's origin.

    }

    private void computeTriangles() {
        triangles.clear();
        // Iterate through the terrain grid and check for intersections
        for (int x = 0; x < heightData.length - 1; x++) {
            for (int z = 0; z < heightData[x].length - 1; z++) {
                // Calculate the vertices of the current terrain quad
                Vector3f vertex0 = new Vector3f(x, heightData[x][z], z);
                Vector3f vertex1 = new Vector3f((x + 1), heightData[x + 1][z], z);
                Vector3f vertex2 = new Vector3f(x, heightData[x][z + 1], (z + 1));

                triangles.add(new TerrainTriangle(vertex0, vertex1, vertex2, new Vector2f(x, z)));
            }
        }
    }

    public Vector3f calculateSlidingDisplacement(Vector3f position) {
        Vector2f gridPosition = getGridPosition(position.x, position.z);
        // Ensure the terrain normal is normalized
        Vector3f terrainNormal = triangles.stream()
                .filter(t -> t.getGridPosition().equals(gridPosition))
                .findFirst().orElseThrow().getNormal();
        terrainNormal = terrainNormal.normalize();

        // Project the player's position vector onto the terrain plane
        float displacementDotNormal = position.dot(terrainNormal);
        Vector3f projectedDisplacement = terrainNormal.mul(displacementDotNormal);

        // Calculate the sliding position by subtracting the projected position
        return position.sub(projectedDisplacement);
    }

    public boolean isCollide(Line line) {
        if (!CollisionDetection.isCollide(line, borders))
            return false;

        // Get the start and end points of the line
        Vector3f start = line.getStart();
        Vector3f end = line.getEnd();
        if ((isPointBlowGround(start) && !isPointBlowGround(end)) ||
                (isPointBlowGround(end) && !isPointBlowGround(start)) ||
                isPointOnGround(start) || isPointOnGround(end))
            return true;

        for (TerrainTriangle triangle : triangles) {
            if (CollisionDetection.isCollide(line, triangle.toTriangle()))
                return true;
        }

        return false;
    }


    private boolean isNotPointInside(Vector3f point) {
        // Check if the point is within the terrain's boundaries (defined by 'borders')
        return !borders.isPointInside(point);
    }
}
