package collisionDetection.broadPhase;

import java.util.Objects;

public class BPPairs {
    private BPBox bpBox1;
    private BPBox bpBox2;

    public BPPairs(BPBox bpBox1, BPBox bpBox2) {
        this.bpBox1 = bpBox1;
        this.bpBox2 = bpBox2;
    }

    public BPBox getBpBox1() {
        return bpBox1;
    }

    public void setBpBox1(BPBox bpBox1) {
        this.bpBox1 = bpBox1;
    }

    public BPBox getBpBox2() {
        return bpBox2;
    }

    public void setBpBox2(BPBox bpBox2) {
        this.bpBox2 = bpBox2;
    }

    @Override
    public String toString() {
        return "BPPairs{" +
                "bpBox1=" + bpBox1 +
                ", bpBox2=" + bpBox2 +
                '}';
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        BPPairs bpPairs = (BPPairs) o;
        return Objects.equals(bpBox1, bpPairs.bpBox1) && Objects.equals(bpBox2, bpPairs.bpBox2);
    }

    @Override
    public int hashCode() {
        return Objects.hash(bpBox1, bpBox2);
    }
}
