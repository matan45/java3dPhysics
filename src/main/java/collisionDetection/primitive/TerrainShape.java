package collisionDetection.primitive;

import math.Const;
import math.Maths;
import math.Vector2f;
import math.Vector3f;

public class TerrainShape {
    //TODO need more work on this
    private float[][] heightData;
    private AABB borders;//define the min and max high
    private static final float SIZE = 1024;
    private float width;//define the width of the terrain
    private float length;//define the length of the terrain
    private float gridSquareSize;

    public TerrainShape(float[][] heightData, AABB borders, Vector3f offset, int width, int length) {
        this.heightData = heightData;
        borders.setMin(borders.getMin().add(offset));
        borders.setMax(borders.getMax().add(offset));
        this.borders = borders;
        this.width = width * SIZE;
        this.length = length * SIZE;
        this.gridSquareSize = SIZE / ((float) heightData.length);
    }

    public void setHeightData(float[][] heightData) {
        this.heightData = heightData;
        this.gridSquareSize = SIZE / ((float) heightData.length);
    }

    public AABB getBorders() {
        return borders;
    }

    public void setBorders(AABB borders) {
        this.borders = borders;
    }

    public float getWidth() {
        return width;
    }

    public void setWidth(float width) {
        this.width = width * SIZE;
    }

    public float getLength() {
        return length;
    }

    public void setLength(float length) {
        this.length = length * SIZE;
    }

    private float getHeightOfTerrain(float worldX, float worldZ) {
        float terrainX = worldX - width;
        float terrainZ = worldZ - length;
        int gridX = (int) Math.floor(terrainX / gridSquareSize);
        int gridZ = (int) Math.floor(terrainZ / gridSquareSize);
        if (gridX >= heightData.length - 1 || gridZ >= heightData.length - 1 || gridX < 0 || gridZ < 0)
            return 0;
        float xCord = (terrainX % gridSquareSize) / gridSquareSize;
        float zCord = (terrainZ % gridSquareSize) / gridSquareSize;

        if (xCord <= (1 - zCord)) {
            return Maths.barryCentric(new Vector3f(0, heightData[gridX][gridZ], 0),
                    new Vector3f(1, heightData[gridX + 1][gridZ], 0), new Vector3f(0, heightData[gridX][gridZ + 1], 1),
                    new Vector2f(xCord, zCord));
        }

        return Maths.barryCentric(new Vector3f(1, heightData[gridX + 1][gridZ], 0),
                new Vector3f(1, heightData[gridX + 1][gridZ + 1], 1), new Vector3f(0, heightData[gridX][gridZ + 1], 1),
                new Vector2f(xCord, zCord));

    }

    public boolean isPointOnGround(Vector3f position) {
        if (!borders.isPointInside(position))
            return false;
        float groundPosition = getHeightOfTerrain(position.x, position.y);
        return position.y - groundPosition < Const.EPSILON;
    }

    public boolean isPointBlowGround(Vector3f position) {
        if (!borders.isPointInside(position))
            return false;
        float groundPosition = getHeightOfTerrain(position.x, position.y);
        return position.y < groundPosition;
    }

    public Vector3f rayTerrainIntersection(Ray ray) {
        if (!isPointInside(ray.getOrigin()))
            return null;

        // Iterate through terrain grid cells
        for (int x = 0; x < heightData.length - 1; x++) {
            for (int z = 0; z < heightData[x].length - 1; z++) {
                float cellX = x * gridSquareSize;
                float cellZ = z * gridSquareSize;

                // Define the vertices of the current grid cell
                Vector3f v1 = new Vector3f(cellX, heightData[x][z], cellZ);
                Vector3f v2 = new Vector3f(cellX + gridSquareSize, heightData[x + 1][z], cellZ);
                Vector3f v3 = new Vector3f(cellX, heightData[x][z + 1], cellZ + gridSquareSize);

                // Perform ray-triangle intersection test
                Vector3f intersectionPoint = rayTriangleIntersection(ray, v1, v2, v3);

                if (intersectionPoint != null) {
                    // Check if intersection point is within terrain bounds
                    if (borders.isPointInside(intersectionPoint)) {
                        // Calculate the height of the terrain at the intersection point
                        float terrainHeight = getHeightOfTerrain(intersectionPoint.x, intersectionPoint.z);

                        // Compare intersection point's height with terrain height
                        if (ray.getOrigin().y < terrainHeight && intersectionPoint.y > terrainHeight) {
                            return intersectionPoint;
                        }
                    }
                }
            }
        }

        return null; // No collision
    }

    private Vector3f rayTriangleIntersection(Ray ray,
                                             Vector3f v1, Vector3f v2, Vector3f v3) {
        Vector3f edge1 = v2.sub(v1);
        Vector3f edge2 = v3.sub(v1);
        Vector3f h = ray.getDirection().cross(edge2);
        float a = edge1.dot(h);

        if (a > -Const.EPSILON && a < Const.EPSILON) {
            return null; // Ray is parallel to the triangle
        }

        float f = 1.0f / a;
        Vector3f s = ray.getOrigin().sub(v1);
        float u = f * s.dot(h);

        if (u < 0.0f || u > 1.0f) {
            return null;
        }

        Vector3f q = s.cross(edge1);
        float v = f * ray.getDirection().dot(q);

        if (v < 0.0f || u + v > 1.0f) {
            return null;
        }

        float t = f * edge2.dot(q);

        if (t > Const.EPSILON) {
            return ray.getOrigin().add(ray.getDirection().mul(t));
        }

        return null; // Ray intersects triangle behind origin
    }

    private Vector3f calculateTerrainNormal(float x, float z) {
        // Get neighboring heights for the cross product calculation
        float leftHeight = getHeightOfTerrain(x - 1, z);
        float rightHeight = getHeightOfTerrain(x + 1, z);
        float downHeight = getHeightOfTerrain(x, z - 1);
        float upHeight = getHeightOfTerrain(x, z + 1);

        // Calculate the tangent and bitangent vectors
        Vector3f tangent = new Vector3f(2.0f, rightHeight - leftHeight, 0.0f);
        Vector3f bitangent = new Vector3f(0.0f, upHeight - downHeight, 2.0f);

        // Calculate the terrain normal using the cross product of tangent and bitangent
        Vector3f terrainNormal = tangent.cross(bitangent);
        terrainNormal.normalize();

        return terrainNormal;
    }

    public Vector3f calculateSlidingDisplacement(Vector3f displacement) {
        // Ensure the terrain normal is normalized
        Vector3f terrainNormal = calculateTerrainNormal(displacement.x, displacement.y);
        terrainNormal.normalize();

        // Project the player's displacement vector onto the terrain plane
        float displacementDotNormal = displacement.dot(terrainNormal);
        Vector3f projectedDisplacement = terrainNormal.mul(displacementDotNormal);

        // Calculate the sliding displacement by subtracting the projected displacement
        return displacement.sub(projectedDisplacement);
    }


    private boolean isPointInside(Vector3f point) {
        // Check if the point is within the terrain's boundaries (defined by 'borders')
        return borders.isPointInside(point);
    }
}
