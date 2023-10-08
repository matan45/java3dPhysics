package collisionDetection.util.quickHull;

public class QHUtil {
    public static Face create(Vertex[] vtxArray, int[] indices) {
        Face face = new Face();
        HalfEdge hePrev = null;
        for (int index : indices) {
            HalfEdge he = new HalfEdge(vtxArray[index], face);
            if (hePrev != null) {
                he.setPrev(hePrev);
                hePrev.setNext(he);
            } else {
                face.he0 = he;
            }
            hePrev = he;
        }
        face.he0.setPrev(hePrev);
        assert hePrev != null;
        hePrev.setNext(face.he0);

        // compute the normal and offset
        face.computeNormalAndCentroid();
        return face;
    }


    public static Face createTriangle(Vertex v0, Vertex v1, Vertex v2) {
        Face face = new Face();
        HalfEdge he0 = new HalfEdge(v0, face);
        HalfEdge he1 = new HalfEdge(v1, face);
        HalfEdge he2 = new HalfEdge(v2, face);

        he0.prev = he2;
        he0.next = he1;
        he1.prev = he0;
        he1.next = he2;
        he2.prev = he1;
        he2.next = he0;

        face.he0 = he0;

        // compute the normal and offset
        face.computeNormalAndCentroid(0);
        return face;
    }
}
