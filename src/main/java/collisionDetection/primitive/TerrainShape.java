package collisionDetection.primitive;

import math.Const;
import math.Maths;
import math.Vector2f;
import math.Vector3f;

public class TerrainShape {

    private float[][] heightData;//TODO splice for quad tree
    private AABB borders;//define the min and max high
    private static final float SIZE = 1024;
    private float width;//define the width of the terrain
    private float length;//define the length of the terrain
    private float gridSquareSize;
    //add also ray cast


    public TerrainShape(float[][] heightData, AABB borders, int width, int length) {
        this.heightData = heightData;
        this.borders = borders;
        this.width = width * SIZE;
        this.length = length * SIZE;
        this.gridSquareSize = SIZE / ((float) heightData.length - 1);
    }

    public void setHeightData(float[][] heightData) {
        this.heightData = heightData;
        this.gridSquareSize = SIZE / ((float) heightData.length - 1);
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

    public float getHeightOfTerrain(float worldX, float worldZ) {
        float terrainX = worldX - this.width;
        float terrainZ = worldZ - this.length;
        int gridX = (int) Math.floor(terrainX / gridSquareSize);
        int gridZ = (int) Math.floor(terrainZ / gridSquareSize);
        if (gridX >= heightData.length - 1 || gridZ >= heightData.length - 1 || gridX < 0 || gridZ < 0)
            return 0;
        float xCord = (terrainX % gridSquareSize) / gridSquareSize;
        float zCord = (terrainZ % gridSquareSize) / gridSquareSize;
        float answer;
        if (xCord <= (1 - zCord)) {
            answer = Maths.barryCentric(new Vector3f(0, heightData[gridX][gridZ], 0),
                    new Vector3f(1, heightData[gridX + 1][gridZ], 0), new Vector3f(0, heightData[gridX][gridZ + 1], 1),
                    new Vector2f(xCord, zCord));
        } else {
            answer = Maths.barryCentric(new Vector3f(1, heightData[gridX + 1][gridZ], 0),
                    new Vector3f(1, heightData[gridX + 1][gridZ + 1], 1), new Vector3f(0, heightData[gridX][gridZ + 1], 1),
                    new Vector2f(xCord, zCord));
        }
        return answer;

    }

    public boolean isPointOnGround(Vector3f position) {
        if (!borders.isPointInside(position))
            return false;
        float groundPosition = getHeightOfTerrain(position.x, position.y);
        return position.y - groundPosition < Const.EPSILON;
    }

    public Vector3f rayTerrainIntersection(Ray ray) {
        Vector3f rayOrigin = ray.getOrigin();
        Vector3f rayDirection = ray.getDirection();

        // Iterate through terrain grid cells
        for (int x = 0; x < heightData.length - 1; x++) {
            for (int z = 0; z < heightData[0].length - 1; z++) {
                float cellX = x * gridSquareSize;
                float cellZ = z * gridSquareSize;

                // Define the vertices of the current grid cell
                Vector3f v1 = new Vector3f(cellX, heightData[x][z], cellZ);
                Vector3f v2 = new Vector3f(cellX + gridSquareSize, heightData[x + 1][z], cellZ);
                Vector3f v3 = new Vector3f(cellX, heightData[x][z + 1], cellZ + gridSquareSize);

                // Perform ray-triangle intersection test
                Vector3f intersectionPoint = rayTriangleIntersection(rayOrigin, rayDirection, v1, v2, v3);

                if (intersectionPoint != null) {
                    // Check if intersection point is within terrain bounds
                    if (borders.isPointInside(intersectionPoint)) {
                        // Calculate the height of the terrain at the intersection point
                        float terrainHeight = getHeightOfTerrain(intersectionPoint.x, intersectionPoint.z);

                        // Compare intersection point's height with terrain height
                        if (rayOrigin.y < terrainHeight && intersectionPoint.y > terrainHeight) {
                            return intersectionPoint;
                        }
                    }
                }
            }
        }

        return null; // No collision
    }

    private Vector3f rayTriangleIntersection(Vector3f rayOrigin, Vector3f rayDirection,
                                             Vector3f v1, Vector3f v2, Vector3f v3) {
        Vector3f edge1 = v2.sub(v1);
        Vector3f edge2 = v3.sub(v1);
        Vector3f h = rayDirection.cross(edge2);
        float a = edge1.dot(h);

        if (a > -Const.EPSILON && a < Const.EPSILON) {
            return null; // Ray is parallel to the triangle
        }

        float f = 1.0f / a;
        Vector3f s = rayOrigin.sub(v1);
        float u = f * s.dot(h);

        if (u < 0.0f || u > 1.0f) {
            return null;
        }

        Vector3f q = s.cross(edge1);
        float v = f * rayDirection.dot(q);

        if (v < 0.0f || u + v > 1.0f) {
            return null;
        }

        float t = f * edge2.dot(q);

        if (t > Const.EPSILON) {
            return rayOrigin.add(rayDirection.mul(t));
        }

        return null; // Ray intersects triangle behind origin
    }


}
