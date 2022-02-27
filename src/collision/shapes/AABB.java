package collision.shapes;

import math.Vector3f;

public class AABB {
    // Maximum world coordinates of the AABB on the x,y and z axis
    private final Vector3f mMaxCoordinates;

    // Minimum world coordinates of the AABB on the x,y and z axis
    private final Vector3f mMinCoordinates;

    // Constructor
    public AABB() {
        mMaxCoordinates = new Vector3f();
        mMinCoordinates = new Vector3f();
    }

    // Constructor
    public AABB(Vector3f minCoordinates, Vector3f maxCoordinates) {
        mMaxCoordinates = maxCoordinates;
        mMinCoordinates = minCoordinates;
    }

    // Return the center point of the AABB in world coordinates
    public Vector3f getCenter() {
        return new Vector3f(mMinCoordinates).add(mMaxCoordinates).multiply(0.5f);
    }

    // Return the maximum coordinates of the AABB
    public Vector3f getMax() {
        return mMaxCoordinates;
    }

    // Set the maximum coordinates of the AABB
    public void setMax(Vector3f max) {
        mMaxCoordinates.set(max);
    }

    // Return the minimum coordinates of the AABB
    public Vector3f getMin() {
        return mMinCoordinates;
    }

    // Set the minimum coordinates of the AABB
    public void setMin(Vector3f min) {
        mMinCoordinates.set(min);
    }

    // Return true if the current AABB is overlapping with the AABB in argument.
    // Two AABBs overlap if they overlap in the three x, y and z axis at the same time
    public boolean testCollision(AABB aabb) {
        if (mMaxCoordinates.getX() < aabb.mMinCoordinates.getX() || aabb.mMaxCoordinates.getX() < mMinCoordinates.getX()) {
            return false;
        }
        if (mMaxCoordinates.getZ() < aabb.mMinCoordinates.getZ() || aabb.mMaxCoordinates.getZ() < mMinCoordinates.getZ()) {
            return false;
        }
        return mMaxCoordinates.getY() >= aabb.mMinCoordinates.getY() && aabb.mMaxCoordinates.getY() >= mMinCoordinates.getY();
    }
}
