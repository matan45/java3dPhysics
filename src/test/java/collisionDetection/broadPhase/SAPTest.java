package collisionDetection.broadPhase;

import collisionDetection.primitive.Sphere;
import math.Vector3f;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import static org.junit.jupiter.api.Assertions.assertEquals;

class SAPTest {

    private SAP sap;
    private List<BPBox> bpBoxes;
    private BPBox bpBox1;

    @BeforeEach
    void setUp() {
        bpBoxes = new ArrayList<>();

        Sphere sphere1 = new Sphere(new Vector3f(), 1.0f);
        bpBox1 = new BPBox(new Vector3f(-1, -1, 0), new Vector3f(), sphere1);

        Sphere sphere2 = new Sphere(new Vector3f(1, 1, 1), 1.0f);
        BPBox bpBox2 = new BPBox(new Vector3f(), new Vector3f(1, 1, 0), sphere2);

        Sphere sphere3 = new Sphere(new Vector3f(2, 2, 2), 1.0f);
        BPBox bpBox3 = new BPBox(new Vector3f(0.5f, 0.5f, 0), new Vector3f(1, 1, 0), sphere3);

        Sphere sphere4 = new Sphere(new Vector3f(3, 3, 3), 1.0f);
        BPBox bpBox4 = new BPBox(new Vector3f(3, 3, 0), new Vector3f(4, 4, 0), sphere4);

        Sphere sphere5 = new Sphere(new Vector3f(4, 4, 4), 1.0f);
        BPBox bpBox5 = new BPBox(new Vector3f(6, 6, 0), new Vector3f(6, 6, 0), sphere5);

        bpBoxes.add(bpBox1);
        bpBoxes.add(bpBox2);
        bpBoxes.add(bpBox3);
        bpBoxes.add(bpBox4);
        bpBoxes.add(bpBox5);
        sap = new SAP();
    }

    @Test
    public void testInsertAndQueryAndRemoveAll() {
        sap.addAll(bpBoxes);
        Set<BPPairs> pairsSet = sap.query();
        assertEquals(2, pairsSet.size());
        sap.removeAll(bpBoxes);
        Set<BPPairs> pairsSet2 = sap.query();
        assertEquals(0, pairsSet2.size());
    }

    @Test
    public void testClear() {
        sap.addAll(bpBoxes);
        sap.clear();
        Set<BPPairs> pairsSet = sap.query();
        assertEquals(0, pairsSet.size());
    }

    @Test
    public void testQueryObject() {
        Sphere sphere1 = new Sphere(new Vector3f(8, 8, 0), 1.0f);
        BPBox bpBox1 = new BPBox(new Vector3f(-2, -2, 0), new Vector3f(2, 2, 0), sphere1);
        sap.addAll(bpBoxes);
        Set<BPBox> pairsSet = sap.query(bpBox1);
        assertEquals(3, pairsSet.size());
    }

    @Test
    public void testUpdate() {
        sap.addAll(bpBoxes);
        bpBox1.setMax(new Vector3f(3, 5, 0));
        sap.update(bpBox1);
        Set<BPPairs> pairsSet = sap.query();
        assertEquals(3, pairsSet.size());
    }

}