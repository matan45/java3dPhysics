package collisionDetection.primitive;

public class Interval {
    private float min;
    private float max;

    public Interval() {
        min = 0;
        max = 0;
    }

    public Interval(float min, float max) {
        this.min = min;
        this.max = max;
    }

    public float getMin() {
        return min;
    }

    public void setMin(float min) {
        this.min = min;
    }

    public float getMax() {
        return max;
    }

    public void setMax(float max) {
        this.max = max;
    }

    @Override
    public String toString() {
        return "ItInterval{" +
                "min=" + min +
                ", max=" + max +
                '}';
    }
}
