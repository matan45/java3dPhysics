package collisionDetection.primitive;

import org.joml.Vector3f;

public record CollisionResult(boolean isCollide, double minDistance, Vector3f closestPoint1,Vector3f closestPoint2) { }
