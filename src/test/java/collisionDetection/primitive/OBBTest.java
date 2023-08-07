package collisionDetection.primitive;

import org.joml.Vector3f;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class OBBTest {
    /*
          public void rotate(float thetaX, float thetaY, float thetaZ) {
         // Step 2: Create rotation matrices
         float cosX = (float) Math.cos(thetaX);
         float sinX = (float) Math.sin(thetaX);
         Matrix3f Rx = new Matrix3f(1, 0, 0, 0, cosX, -sinX, 0, sinX, cosX);

         float cosY = (float) Math.cos(thetaY);
         float sinY = (float) Math.sin(thetaY);
         Matrix3f Ry = new Matrix3f(cosY, 0, sinY, 0, 1, 0, -sinY, 0, cosY);

         float cosZ = (float) Math.cos(thetaZ);
         float sinZ = (float) Math.sin(thetaZ);
         Matrix3f Rz = new Matrix3f(cosZ, -sinZ, 0, sinZ, cosZ, 0, 0, 0, 1);

         // Step 3: Update the axes
         axes[0] = Rx.transform(axes[0]);
         axes[1] = Ry.transform(axes[1]);
         axes[2] = Rz.transform(axes[2]);

         // Step 4: Update the corners
         float[] halfLengths = getHalfLengths();
         for (int i = 0; i < 8; i++) {
             float x = center.x + axes[0].x * ((i & 1) == 0 ? halfLengths[0] : -halfLengths[0]);
             float y = center.y + axes[1].y * ((i & 2) == 0 ? halfLengths[1] : -halfLengths[1]);
             float z = center.z + axes[2].z * ((i & 4) == 0 ? halfLengths[2] : -halfLengths[2]);
             corners[i] = new Vector3f(x, y, z);
         }
     }
         */
    @Test
    void testIsOBBCollidingWithOBB() {
        Vector3f center1 = new Vector3f(0, 0, 0);
        float[] halfLengths1 = {0.3f, 0.3f, 0.3f};
        OBB obb1 = new OBB(center1, halfLengths1);

        Vector3f center2 = new Vector3f(0.4f, 0.4f, 0.4f);
        float[] halfLengths2 = {0.1f, 0.1f, 0.1f};
        OBB obb2 = new OBB(center2, halfLengths2);

        CollisionResult expected = new CollisionResult(true, 0.0, new Vector3f(0.3f, 0.3f, 0.3f), new Vector3f(0.3f, 0.3f, 0.3f));
        CollisionResult result = OBB.isOBBCollidingWithOBB(obb1, obb2);

        assertEquals(expected, result);
    }

    @Test
    void testIsOBBNoCollidingWithOBB() {
        Vector3f center1 = new Vector3f(0, 0, 0);
        float[] halfLengths1 = {1, 1, 1};
        OBB obb1 = new OBB(center1, halfLengths1);

        Vector3f center2 = new Vector3f(3, 3, 3);
        float[] halfLengths2 = {1, 1, 1};
        OBB obb2 = new OBB(center2, halfLengths2);

        CollisionResult expected = new CollisionResult(false, 1.7320508075688772, new Vector3f(1, 1, 1), new Vector3f(0, 0, 0));
        CollisionResult result = OBB.isOBBCollidingWithOBB(obb1, obb2);

        assertEquals(expected, result);
    }
}
