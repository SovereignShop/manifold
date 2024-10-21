package manifold3d;

import org.bytedeco.javacpp.*;
import org.bytedeco.javacpp.annotation.*;

import manifold3d.LibraryPaths;
import manifold3d.manifold.CrossSectionVector;
import manifold3d.manifold.CrossSection;
import manifold3d.glm.DoubleVec3Vector;
import manifold3d.glm.DoubleMat4x3Vector;
import manifold3d.pub.PolygonsVector;
import manifold3d.pub.Polygons;
import manifold3d.pub.SimplePolygon;
import manifold3d.UIntVecVector;

import manifold3d.Manifold;
import java.nio.DoubleBuffer;
import java.nio.IntBuffer;

@Platform(compiler = "cpp17", include = {"mesh_utils.hpp", "buffer_utils.hpp"}, linkpath = { LibraryPaths.MANIFOLD_LIB_DIR, LibraryPaths.MANIFOLD_LIB_DIR_WINDOWS }, link = {"manifold"})
public class MeshUtils extends Pointer {
    static { Loader.load(); }

    public MeshUtils() { }

    public static enum LoftAlgorithm {
        EagerNearestNeighbor(0),
        Isomorphic(1);

        public final long value;
        private LoftAlgorithm(long v) { this.value = v; }
    }

    public static native @ByVal Manifold Polyhedron(DoublePointer vertices, @Cast("std::size_t") long nVertices, IntPointer faceBuf, IntPointer faceLengths, @Cast("std::size_t") long nFaces);
    public static Manifold PolyhedronFromBuffers(DoubleBuffer vertices, long nVertices, IntBuffer faceBuf, IntBuffer faceLengths, long nFaces) {

        DoublePointer verticesPtr = new DoublePointer(vertices);
        IntPointer faceBufPtr = new IntPointer(faceBuf);
        IntPointer lengthsPtr = new IntPointer(faceLengths);

        return Polyhedron(verticesPtr, nVertices, faceBufPtr, lengthsPtr, nFaces);
    }

    public static native @ByVal Manifold CreateSurface(@Const DoublePointer heightMap, int width, int height);
    public static native @ByVal Manifold CreateSurface(@Const DoublePointer heightMap, int width, int height, double pixelWidth);
    public static Manifold CreateSurface(double[] heightMapArray, int width, int height) {
        DoublePointer heightMapPtr = new DoublePointer(heightMapArray);
        return CreateSurface(heightMapPtr, width, height);
    }
    public static Manifold CreateSurface(double[] heightMapArray, int width, int height, double pixelWidth) {
        DoublePointer heightMapPtr = new DoublePointer(heightMapArray);
        return CreateSurface(heightMapPtr, width, height, pixelWidth);
    }

    public static Manifold CreateSurface(DoubleBuffer heightMapBuffer, int width, int height) {
        DoublePointer heightMapPtr = new DoublePointer(heightMapBuffer);
        return CreateSurface(heightMapPtr, width, height);
    }
    public static Manifold CreateSurface(DoubleBuffer heightMapBuffer, int width, int height, double pixelWidth) {
        DoublePointer heightMapPtr = new DoublePointer(heightMapBuffer);
        return CreateSurface(heightMapPtr, width, height, pixelWidth);
    }


    public static native @ByVal Manifold Loft(@ByRef SimplePolygon polygon, @ByRef DoubleMat4x3Vector transforms);
    public static native @ByVal Manifold Loft(@ByRef SimplePolygon polygon, @ByRef DoubleMat4x3Vector transforms, LoftAlgorithm algorithmEnum);
    public static native @ByVal Manifold Loft(@ByRef Polygons polygons, @ByRef DoubleMat4x3Vector transforms);
    public static native @ByVal Manifold Loft(@ByRef Polygons polygons, @ByRef DoubleMat4x3Vector transforms, LoftAlgorithm algorithmEnum);
    public static native @ByVal Manifold Loft(@ByRef PolygonsVector polygons, @ByRef DoubleMat4x3Vector transforms);
    public static native @ByVal Manifold Loft(@ByRef PolygonsVector polygons, @ByRef DoubleMat4x3Vector transforms, LoftAlgorithm algorithmEnum);
    public static native @ByVal Manifold Loft(@ByRef CrossSectionVector sections, @ByRef DoubleMat4x3Vector transforms);
    public static native @ByVal Manifold Loft(@ByRef CrossSectionVector sections, @ByRef DoubleMat4x3Vector transforms, LoftAlgorithm algorithmEnum);
    public static native @ByVal Manifold Loft(@ByRef CrossSection section, @ByRef DoubleMat4x3Vector transforms);
    public static native @ByVal Manifold Loft(@ByRef CrossSection section, @ByRef DoubleMat4x3Vector transforms, LoftAlgorithm algorithmEnum);
}
