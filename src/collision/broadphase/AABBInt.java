package collision.broadphase;

import collision.shapes.AABB;

import static math.Mathematics.encodeFloatIntoInteger;

public class AABBInt {
    private final long[] min = new long[3];
    private final long[] max = new long[3];

    private long[] getMin() {
        return min;
    }

    private long[] getMax() {
        return max;
    }

    private AABBInt(AABB aabb) {
        min[0] = encodeFloatIntoInteger(aabb.getMin().getX());
        min[1] = encodeFloatIntoInteger(aabb.getMin().getY());
        min[2] = encodeFloatIntoInteger(aabb.getMin().getZ());
        max[0] = encodeFloatIntoInteger(aabb.getMax().getX());
        max[1] = encodeFloatIntoInteger(aabb.getMax().getY());
        max[2] = encodeFloatIntoInteger(aabb.getMax().getZ());
    }

    private AABBInt(long minValue, long maxValue) {
        min[0] = minValue;
        min[1] = minValue;
        min[2] = minValue;
        max[0] = maxValue;
        max[1] = maxValue;
        max[2] = maxValue;
    }
}
