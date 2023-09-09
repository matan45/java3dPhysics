package collisionDetection.broadPhase;

import collisionDetection.primitive.Ray;

import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

public class BVH implements BroadPhase {

    private BVHNode root;

    private static final int MAX_OBJECTS_PER_LEAF = 10;

    public BVH() {
        root = null;
    }

    @Override
    public void insert(BPBox obj) {
        // Check if the root is null
        if (root == null) {
            // Create a new BVHNode with the object's bounding box as the node's bounds
            root = new BVHNode();
            root.getObjects().add(obj); // Add the object to the root node's objects list
            root.setBounds(obj); // Set the root node's bounds to the object's bounds
        } else {
            // Recursively insert the object into the BVH tree starting from the root
            insertRecursive(root, obj);
        }
    }

    @Override
    public void remove(BPBox obj) {
        // Check if the root is null
        if (root == null) {
            // The tree is empty, nothing to remove
            return;
        }

        // Recursively remove the object from the BVH tree starting from the root
        removeRecursive(root, obj);
    }

    @Override
    public void update(BPBox obj) {
        remove(obj);
        insert(obj);
    }

    @Override
    public Set<BPPairs> query() {
        Set<BPPairs> pairs = new HashSet<>();
        // Perform a broad-phase collision detection check recursively starting from the root
        queryRecursive(root, pairs);
        return pairs;
    }

    @Override
    public Set<BPBox> query(Ray ray) {
        Set<BPBox> result = new HashSet<>();

        // Perform the query recursively starting from the root
        queryRecursive(root, ray, result);

        return result;
    }

    @Override
    public Set<BPBox> query(BPBox obj) {
        Set<BPBox> result = new HashSet<>();

        // Perform the query recursively starting from the root
        queryRecursive(root, obj, result);

        return result.stream()
                .filter(b -> !b.getShape().equals(obj.getShape()))
                .collect(Collectors.toSet());
    }

    @Override
    public void clear() {
        root = null;// Set the root node to null to clear the BVH.
    }

    @Override
    public void addAll(List<BPBox> boxes) {
        for (BPBox bpBox : boxes)
            insert(bpBox);
    }

    @Override
    public void updateAll(List<BPBox> boxes) {
        for (BPBox bpBox : boxes)
            update(bpBox);
    }

    @Override
    public void removeAll(List<BPBox> boxes) {
        for (BPBox bpBox : boxes)
            remove(bpBox);
    }

    // Recursive method to insert an object into the BVH tree
    private void insertRecursive(BVHNode node, BPBox obj) {
        if (node.isLeaf()) {
            // Check if the node contains too many objects (e.g., a maximum threshold)
            if (node.getObjects().size() >= MAX_OBJECTS_PER_LEAF) {
                // Split the leaf node into two child nodes along the longest axis
                // and distribute the objects between the left and right child nodes

                // Determine the longest axis of the node's bounding box
                int longestAxis = node.getBounds().getLongestAxis();

                // Create left and right child nodes
                BVHNode leftChild = new BVHNode();
                BVHNode rightChild = new BVHNode();

                // Distribute the objects between the left and right child nodes
                for (BPBox existingObj : node.getObjects()) {
                    if (existingObj.getCenter().get(longestAxis) <= node.getBounds().getCenter().get(longestAxis)) {
                        leftChild.getObjects().add(existingObj);
                        leftChild.setBounds(BVHNode.merge(leftChild.getBounds(), existingObj));
                    } else {
                        rightChild.getObjects().add(existingObj);
                        rightChild.setBounds(BVHNode.merge(rightChild.getBounds(), existingObj));
                    }
                }

                // Clear the objects list in the current node
                node.getObjects().clear();

                // Set the current node as an internal node (no longer a leaf)
                node.setLeaf(false);

                // Set the left and right child nodes
                node.setLeft(leftChild);
                node.setRight(rightChild);

                // Insert the new object into the appropriate child node
                insertRecursive(node, obj);
            } else {
                // The leaf node does not contain too many objects, so simply add the object to it
                node.getObjects().add(obj);
                // Update the node's bounds to include the new object's bounds
                node.setBounds(BVHNode.merge(node.getBounds(), obj));
            }
        } else {
            insertRecursive(node.getLeftChild(), obj);
            insertRecursive(node.getRightChild(), obj);
        }
    }


    // Recursive method to remove an object from the BVH tree
    private void removeRecursive(BVHNode node, BPBox obj) {
        if (node.isLeaf()) {
            // Check if the object exists in the leaf node's objects list
            if (node.getObjects().contains(obj)) {
                // Remove the object from the leaf node's objects list
                node.getObjects().remove(obj);
                // If the node has no more objects, remove it from the tree
                if (node.getObjects().isEmpty()) {
                    // Handle the case where the parent node might become a leaf node
                    handlePossibleMerge(node);
                }
            }
        } else {
            removeRecursive(node.getLeftChild(), obj);
            removeRecursive(node.getRightChild(), obj);
        }
    }

    // Helper method to handle possible merge and tree restructuring
    private void handlePossibleMerge(BVHNode node) {
        if (node == root) {
            // The root node should always remain as the root
            return;
        }

        BVHNode parent = getParentNode(root, node);
        BVHNode sibling = (parent.getLeftChild() == node) ? parent.getRightChild() : parent.getLeftChild();

        // Merge the sibling's objects into the parent node's objects
        parent.getObjects().addAll(sibling.getObjects());
        parent.setBounds(BVHNode.merge(parent.getBounds(), sibling.getBounds()));

        // Remove the sibling node from the tree
        if (parent == root) {
            // The parent node was the root, so make the sibling node the new root
            root = sibling;
        } else {
            // Update the parent's reference to the sibling node
            BVHNode grandparent = getParentNode(root, parent);
            if (grandparent.getLeftChild() == parent) {
                grandparent.setLeft(sibling);
            } else {
                grandparent.setRight(sibling);
            }
        }
    }

    // Helper method to get the parent node of a given node
    private BVHNode getParentNode(BVHNode current, BVHNode child) {
        if (current == null || current.isLeaf()) {
            return null;
        }

        if (current.getLeftChild() == child || current.getRightChild() == child) {
            return current;
        }

        BVHNode leftParent = getParentNode(current.getLeftChild(), child);
        if (leftParent != null) {
            return leftParent;
        }

        return getParentNode(current.getRightChild(), child);
    }


    // Recursive method to perform broad-phase collision detection
    private void queryRecursive(BVHNode node, Set<BPPairs> pairs) {
        if (node == null) {
            return; // Stop if the node is null or a leaf node
        }

        if (node.isLeaf() && node.getObjects().size() > 1) {
            for (int i = 0; i < node.getObjects().size() - 1; i++) {
                if (BPBox.isCollide(node.getObjects().get(i), node.getObjects().get(i + 1)))
                    pairs.add(new BPPairs(node.getObjects().get(i), node.getObjects().get(i + 1)));
            }
        }

        if (!node.isLeaf()) {
            // Check for potential collisions between objects in the left and right child nodes
            for (BPBox box1 : node.getLeftChild().getObjects()) {
                for (BPBox box2 : node.getRightChild().getObjects()) {
                    // Check if box1 and box2 potentially collide
                    if (BPBox.isCollide(box1, box2)) {
                        // Create a BPPairs object to represent the pair and add it to the set
                        pairs.add(new BPPairs(box1, box2));
                    }
                }
            }
        }

        // Recursively check for collisions in the left and right child subtrees
        queryRecursive(node.getLeftChild(), pairs);
        queryRecursive(node.getRightChild(), pairs);
    }

    // Recursive method to perform the query
    private void queryRecursive(BVHNode node, BPBox obj, Set<BPBox> result) {
        if (node == null) {
            return; // Stop if the node is null
        }

        // Check if the node's bounding box and the object's bounding box intersect
        if (BPBox.isCollide(node.getBounds(), obj)) {
            if (node.isLeaf()) {
                // If the node is a leaf, add all objects in the leaf to the result set
                result.addAll(node.getObjects().stream()
                        .filter(box -> BPBox.isCollide(obj, box))
                        .collect(Collectors.toSet()));
            } else {
                // If the node is an internal node, recursively query its children
                queryRecursive(node.getLeftChild(), obj, result);
                queryRecursive(node.getRightChild(), obj, result);
            }
        }
    }

    private void queryRecursive(BVHNode node, Ray ray, Set<BPBox> result) {
        if (node == null) {
            return; // Stop if the node is null
        }

        // Check if the node's bounding box and the object's bounding box intersect
        if (BPBox.isCollide(ray, node.getBounds())) {
            if (node.isLeaf()) {
                // If the node is a leaf, add all objects in the leaf to the result set
                result.addAll(node.getObjects());
            } else {
                // If the node is an internal node, recursively query its children
                queryRecursive(node.getLeftChild(), ray, result);
                queryRecursive(node.getRightChild(), ray, result);
            }
        }
    }


}
