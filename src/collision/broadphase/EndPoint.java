package collision.broadphase;

public class EndPoint {
    // ID of the AABB box corresponding to this end-point
    public int boxID;

    // True if the end-point is a minimum end-point of a box
    public boolean isMin;

    // Value (one dimension coordinate) of the end-point
    public long value;

    // Set the values of the endpoint
    public void setValues(int boxID, boolean isMin, long value) {
        this.boxID = boxID;
        this.isMin = isMin;
        this.value = value;
    }
}
