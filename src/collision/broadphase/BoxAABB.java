package collision.broadphase;

import body.CollisionBody;

public class BoxAABB {
    // Index of the 3 minimum end-points of the AABB over the x,y,z axis
    public final int[] min = new int[3];

    // Index of the 3 maximum end-points of the AABB over the x,y,z axis
    public final int[] max = new int[3];

    // Body that corresponds to the owner of the AABB
    public CollisionBody body;
}
