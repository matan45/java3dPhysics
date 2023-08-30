package collisionDetection.narrowPhase;

import collisionDetection.primitive.*;

public class CollisionDetection {
    public static boolean isCollide(Line line, AABB aabb){
        return false;
    }
    public static boolean isCollide(Line line, OBB obb){
        return false;
    }
    public static boolean isCollide(Line line, ConvexPolyhedron convexPolyhedron){
        return false;
    }
    public static boolean isCollide(Line line, Capsule capsule){
        return false;
    }
    public static boolean isCollide(Line line, Cylinder cylinder){
        return false;
    }
    public static boolean isCollide(Line line, Sphere sphere){
        return false;
    }
    public static boolean isCollide(Line line, Triangle triangle){
        return false;
    }
    public static boolean isCollide(Line line, Plane plane){
        return false;
    }
}
