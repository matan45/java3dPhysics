package collisionDetection.broadPhase;

import collisionDetection.primitive.Ray;
import math.Vector3f;

import java.util.Set;

public class BVHNode {
    private BPBox box;
    private BVHNode left;
    private BVHNode right;

    public BVHNode(BPBox box) {
        this.box = box;
        this.left = null;
        this.right = null;
    }

    public void insert(BPBox obj) {
        if (box == null) {
            box = obj;
        } else {
            if (box.equals(obj)) {
                return; // Object already exists, no need to insert it again.
            }
            // Calculate the center of the current node's bounding box.
            Vector3f nodeCenter = box.getCenter();

            // Determine which child node (left or right) to insert the object into.
            boolean insertLeft = (obj.getMin().x + obj.getMax().x < nodeCenter.x) ||
                    (obj.getMin().y + obj.getMax().y < nodeCenter.y) ||
                    (obj.getMin().z + obj.getMax().z < nodeCenter.z);

            if (insertLeft) {
                if (left == null) {
                    left = new BVHNode(obj);
                } else {
                    left.insert(obj);
                }
            } else {
                if (right == null) {
                    right = new BVHNode(obj);
                } else {
                    right.insert(obj);
                }
            }

            performBalancing(obj);

            // Update the current node's bounding box to include the inserted object.
            // You need to implement a method to update the bounding box accordingly.
            // For example, you can extend the bounding box to include both the current
            // box and the inserted object's bounding box.
            box = updateBoundingBox(box, obj);
        }
    }

    private BPBox updateBoundingBox(BPBox currentBox, BPBox insertedBox) {
        // Get the minimum and maximum coordinates of the current bounding box.
        Vector3f currentMin = currentBox.getMin();
        Vector3f currentMax = currentBox.getMax();

        // Get the minimum and maximum coordinates of the inserted object's bounding box.
        Vector3f insertedMin = insertedBox.getMin();
        Vector3f insertedMax = insertedBox.getMax();

        // Compute the new minimum and maximum coordinates for the updated bounding box.
        float newMinX = Math.min(currentMin.x, insertedMin.x);
        float newMinY = Math.min(currentMin.y, insertedMin.y);
        float newMinZ = Math.min(currentMin.z, insertedMin.z);

        float newMaxX = Math.max(currentMax.x, insertedMax.x);
        float newMaxY = Math.max(currentMax.y, insertedMax.y);
        float newMaxZ = Math.max(currentMax.z, insertedMax.z);

        // Create and return the updated bounding box.
        Vector3f newMin = new Vector3f(newMinX, newMinY, newMinZ);
        Vector3f newMax = new Vector3f(newMaxX, newMaxY, newMaxZ);

        return new BPBox(newMin, newMax, currentBox.getShape());
    }

    public void query(Set<BPPairs> collisionPairs) {
        // Check for potential collisions between objects in this node and its children.
        if (left != null && right != null) {
            if (BPBox.isCollide(left.box, right.box)) {
                collisionPairs.add(new BPPairs(left.box, right.box));
            }
        }

        // Recursively check for collisions in the left and right children.
        if (left != null) {
            left.query(collisionPairs);
        }
        if (right != null) {
            right.query(collisionPairs);
        }
    }

    public void remove(BPBox obj) {
        // Check if the current node's bounding box matches the object to remove.
        if (box == obj) {
            box = null; // Remove the object from this node.
        } else {
            // Check if the object to remove potentially collides with the left child node.
            boolean isCollideWithLeft = left != null && BPBox.isCollide(left.box, obj);

            // Check if the object to remove potentially collides with the right child node.
            boolean isCollideWithRight = right != null && BPBox.isCollide(right.box, obj);

            // Recursively remove the object from the appropriate child node(s).
            if (isCollideWithLeft || isCollideWithRight) {
                if (isCollideWithLeft) {
                    left.remove(obj);
                }
                if (isCollideWithRight) {
                    right.remove(obj);
                }

                performBalancing(obj);

                // After removing, update the current node's bounding box.
                updateBoundingBox();
            }
        }
    }

    // Method to update the bounding box of the current node.
    private void updateBoundingBox() {
        // Initialize the bounding box with the current node's box.
        BPBox newBoundingBox = box;

        // Update the bounding box based on the left child's bounding box, if it exists.
        if (left != null) {
            newBoundingBox = combineBoundingBoxes(newBoundingBox, left.box);
        }

        // Update the bounding box based on the right child's bounding box, if it exists.
        if (right != null) {
            newBoundingBox = combineBoundingBoxes(newBoundingBox, right.box);
        }

        // Assign the updated bounding box to the current node.
        box = newBoundingBox;
    }

    public void query(BPBox obj, Set<BPBox> potentialCollisions) {
        if (BPBox.isCollide(box, obj)) {
            // If there's a potential collision with the current node's box and the query object,
            // add the current node's box to the set of potential collisions.
            potentialCollisions.add(box);
        }

        if (left != null && BPBox.isCollide(left.box, obj)) {
            // If there's a potential collision with the left child node,
            // recursively query the left child node.
            left.query(obj, potentialCollisions);
        }

        if (right != null && BPBox.isCollide(right.box, obj)) {
            // If there's a potential collision with the right child node,
            // recursively query the right child node.
            right.query(obj, potentialCollisions);
        }
    }

    public void query(Ray ray, Set<BPBox> potentialCollisions) {
        if (BPBox.isCollide(ray, box)) {
            // If there's a potential collision with the current node's box and the query object,
            // add the current node's box to the set of potential collisions.
            potentialCollisions.add(box);
        }

        if (left != null && BPBox.isCollide(ray, left.box)) {
            // If there's a potential collision with the left child node,
            // recursively query the left child node.
            left.query(ray, potentialCollisions);
        }

        if (right != null && BPBox.isCollide(ray, right.box)) {
            // If there's a potential collision with the right child node,
            // recursively query the right child node.
            right.query(ray, potentialCollisions);
        }
    }

    // Helper method to combine two bounding boxes into a new one.
    private BPBox combineBoundingBoxes(BPBox box1, BPBox box2) {
        // Calculate the minimum and maximum coordinates of the combined bounding box.
        Vector3f newMin = Vector3f.min(box1.getMin(), box2.getMin());
        Vector3f newMax = Vector3f.max(box1.getMax(), box2.getMax());

        // Create and return the combined bounding box.
        return new BPBox(newMin, newMax, box1.getShape()); // Assuming the shape remains the same.
    }

    private void performBalancing(BPBox obj){
        // Check and perform balancing if necessary.
        if (leftDepth() - rightDepth() > 1) {
            if (obj.getMin().x + obj.getMax().x < box.getCenter().x) {
                // Left-left case, perform a right rotation.
                rotateRight();
            } else {
                // Left-right case, perform a left rotation followed by a right rotation.
                left = left.rotateLeft();
                rotateRight();
            }
        } else if (rightDepth() - leftDepth() > 1) {
            if (obj.getMin().x + obj.getMax().x >= box.getCenter().x) {
                // Right-right case, perform a left rotation.
                rotateLeft();
            } else {
                // Right-left case, perform a right rotation followed by a left rotation.
                right = right.rotateRight();
                rotateLeft();
            }
        }
    }

    private BVHNode rotateLeft() {
        BVHNode newRoot = right;
        right = newRoot.left;
        newRoot.left = this;

        // Update bounding boxes of this and new root nodes.
        updateBoundingBox();
        newRoot.updateBoundingBox();

        return newRoot;
    }

    private BVHNode rotateRight() {
        BVHNode newRoot = left;
        left = newRoot.right;
        newRoot.right = this;

        // Update bounding boxes of this and new root nodes.
        updateBoundingBox();
        newRoot.updateBoundingBox();

        return newRoot;
    }

    // Helper method to calculate the depth of the left subtree.
    private int leftDepth() {
        if (left == null) {
            return 0;
        }
        return 1 + Math.max(left.leftDepth(), left.rightDepth());
    }

    // Helper method to calculate the depth of the right subtree.
    private int rightDepth() {
        if (right == null) {
            return 0;
        }
        return 1 + Math.max(right.leftDepth(), right.rightDepth());
    }

}
