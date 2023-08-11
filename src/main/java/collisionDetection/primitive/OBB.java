package collisionDetection.primitive;

import org.joml.Matrix4f;
import org.joml.Vector3f;
import org.joml.Vector4f;

public class OBB {

    private Vector3f center;
    private Vector3f[] axis;
    private Vector3f halfExtents;

    public OBB(Vector3f center, Vector3f halfExtents) {
        this.center = center;
        this.axis = new Vector3f[]{new Vector3f(1.0f, 0.0f, 0.0f), new Vector3f(0.0f, 1.0f, 0.0f), new Vector3f(0.0f, 0.0f, 1.0f)};
        this.halfExtents = halfExtents;
    }

    public Vector3f getCenter() {
        return center;
    }

    public void setCenter(Vector3f center) {
        this.center = center;
    }

    public Vector3f[] getAxis() {
        return axis;
    }

    public void setAxis(Vector3f[] axis) {
        this.axis = axis;
    }

    public Vector3f getHalfExtents() {
        return halfExtents;
    }

    public void setHalfExtents(Vector3f halfExtents) {
        this.halfExtents = halfExtents;
    }

    public Vector3f closestPoint(Vector3f point) {
        // Transform the point into the local coordinate system of the OBB
        Matrix4f invTransform = getTransform().invert();
        Vector4f pointLocal = new Vector4f(point, 1.0f);
        pointLocal = invTransform.transform(pointLocal);

        // Calculate the closest point on the OBB's surface to the transformed point
        Vector3f closestLocal = new Vector3f();
        for (int i = 0; i < 3; i++) {
            float dist = pointLocal.dot(new Vector4f(axis[i], 0));
            dist = Math.min(halfExtents.get(i), Math.max(-halfExtents.get(i), dist));
            closestLocal = closestLocal.add(axis[i].mul(dist));
        }

        // Transform the closest local point back into the global coordinate system
        Vector4f closestGlobal = getTransform().transform(new Vector4f(closestLocal, 1.0f));

        return new Vector3f(closestGlobal.x, closestGlobal.y, closestGlobal.z);
    }

    public Matrix4f getTransform() {
        Matrix4f translationMatrix = new Matrix4f().translation(center);

        Matrix4f rotationMatrix = new Matrix4f()
                .rotateX(axis[0].x).rotateY(axis[0].y).rotateZ(axis[0].z)
                .rotateX(axis[1].x).rotateY(axis[1].y).rotateZ(axis[1].z)
                .rotateX(axis[2].x).rotateY(axis[2].y).rotateZ(axis[2].z);

        Matrix4f scaleMatrix = new Matrix4f().scaling(halfExtents);

        return translationMatrix.mul(rotationMatrix).mul(scaleMatrix);
    }

    public static boolean isOBBColliding(OBB obb1, OBB obb2) {

        Vector3f[] test = new Vector3f[15];

        test[0] = obb1.getAxis()[0];
        test[1] = obb1.getAxis()[1];
        test[2] = obb1.getAxis()[2];
        test[3] = obb2.getAxis()[0];
        test[4] = obb2.getAxis()[1];
        test[5] = obb2.getAxis()[2];

        for (int i = 0; i < 3; ++i) {
            test[6 + i * 3] = new Vector3f(test[i]).cross(test[0]);
            test[6 + i * 3 + 1] = new Vector3f(test[i]).cross(test[1]);
            test[6 + i * 3 + 2] = new Vector3f(test[i]).cross(test[2]);
        }

        for (Vector3f vector3f : test) {
            if (!overlapOnAxis(obb1, obb2, new Vector3f(0,1,0))) {
                System.out.println("hit");
                System.out.println(vector3f.toString());
                return false; // Seperating axis found
            }
        }
        return true; // Seperating axis not found
    }

    private static boolean overlapOnAxis(OBB obb1, OBB obb2, Vector3f axis) {
        Interval a = getInterval(obb1, axis);
        Interval b = getInterval(obb2, axis);
        return b.getMin() <= a.getMax() && a.getMin() <= b.getMax();
    }

    private static Interval getInterval(OBB obb, Vector3f axis) {
        Vector3f[] vertex = new Vector3f[8];

        Vector3f C = obb.getCenter(); // OBB Center
        Vector3f E = obb.getHalfExtents(); // OBB Extents

        Vector3f[] A = obb.getAxis();

        vertex[0] = C.add(A[0].mul(E.x)).add(A[1].mul(E.y)).add(A[2].mul(E.z));
        vertex[1] = C.sub(A[0].mul(E.x)).add(A[1].mul(E.y)).add(A[2].mul(E.z));
        vertex[2] = C.add(A[0].mul(E.x)).sub(A[1].mul(E.y)).add(A[2].mul(E.z));
        vertex[3] = C.add(A[0].mul(E.x)).add(A[1].mul(E.y)).sub(A[2].mul(E.z));
        vertex[4] = C.sub(A[0].mul(E.x)).sub(A[1].mul(E.y)).sub(A[2].mul(E.z));
        vertex[5] = C.add(A[0].mul(E.x)).sub(A[1].mul(E.y)).sub(A[2].mul(E.z));
        vertex[6] = C.sub(A[0].mul(E.x)).add(A[1].mul(E.y)).sub(A[2].mul(E.z));
        vertex[7] = C.sub(A[0].mul(E.x)).sub(A[1].mul(E.y)).add(A[2].mul(E.z));


        Interval result = new Interval();
        result.setMin(axis.dot(vertex[0]));
        result.setMax(axis.dot(vertex[0]));

        for (int i = 1; i < 8; ++i) {
            float projection = axis.dot(vertex[i]);
            result.setMin(Math.min(projection, result.getMin()));
            result.setMax(Math.max(projection, result.getMax()));
        }

        return result;
    }

}
