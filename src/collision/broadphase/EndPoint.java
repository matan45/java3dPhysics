package collision.broadphase;

public class EndPoint {
    private int boxID;
    private boolean isMin;
    private long value;

    private int getBoxID() {
        return boxID;
    }

    private void setBoxID(int boxID) {
        this.boxID = boxID;
    }

    private boolean isMin() {
        return isMin;
    }

    private long getValue() {
        return value;
    }

    private void setValue(long value) {
        this.value = value;
    }

    private void setValues(int boxID, boolean isMin, long value) {
        this.boxID = boxID;
        this.isMin = isMin;
        this.value = value;
    }
}
