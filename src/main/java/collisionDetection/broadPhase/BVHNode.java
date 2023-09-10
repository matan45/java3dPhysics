package collisionDetection.broadPhase;

import math.Vector3f;

import java.util.ArrayList;
import java.util.List;

public class BVHNode {
    private BPBox nodeBounds;
    boolean isLeaf;
    private final List<BPBox> objects;
    private BVHNode left;
    private BVHNode right;

    public BVHNode() {
        nodeBounds = new BPBox(new Vector3f(), new Vector3f(), null);
        isLeaf = true;
        objects = new ArrayList<>();
        left = null;
        right = null;
    }

    public BPBox getBounds() {
        return nodeBounds;
    }

    public void setBounds(BPBox bounds) {
        nodeBounds = bounds;
    }

    public List<BPBox> getObjects() {
        return objects;
    }

    public void setLeaf(boolean leaf) {
        isLeaf = leaf;
    }

    public void setLeft(BVHNode left) {
        this.left = left;
    }

    public void setRight(BVHNode right) {
        this.right = right;
    }

    public BVHNode getLeftChild() {
        return left;
    }

    public BVHNode getRightChild() {
        return right;
    }

    public boolean isLeaf() {
        return isLeaf;
    }

    public static BPBox merge(BPBox box1, BPBox box2) {
        // Compute the new bounding box that contains both input boxes
        Vector3f newMin = new Vector3f(
                Math.min(box1.getMin().x, box2.getMin().x),
                Math.min(box1.getMin().y, box2.getMin().y),
                Math.min(box1.getMin().z, box2.getMin().z)
        );

        Vector3f newMax = new Vector3f(
                Math.max(box1.getMax().x, box2.getMax().x),
                Math.max(box1.getMax().y, box2.getMax().y),
                Math.max(box1.getMax().z, box2.getMax().z)
        );

        // Create a new BPBox representing the merged bounding box
        return new BPBox(newMin, newMax, null); // Replace 'null' with the appropriate Shape object if needed
    }


}
