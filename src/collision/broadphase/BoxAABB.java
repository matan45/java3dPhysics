package collision.broadphase;

import body.CollisionBody;

public class BoxAABB {
    private final int[] min = new int[3];
    private final int[] max = new int[3];
    private CollisionBody body;

    private int[] getMin() {
        return min;
    }

    private void setMin(int i, int v) {
        min[i] = v;
    }

    private int[] getMax() {
        return max;
    }

    private void setMax(int i, int v) {
        max[i] = v;
    }

    private CollisionBody getBody() {
        return body;
    }

    private void setBody(CollisionBody body) {
        this.body = body;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) {
            return true;
        }
        if (!(o instanceof final BoxAABB boxAABB)) {
            return false;
        }
        return !(body != null ? !body.equals(boxAABB.getBody()) : boxAABB.getBody() != null);
    }

    @Override
    public int hashCode() {
        return body != null ? body.hashCode() : 0;
    }
}
