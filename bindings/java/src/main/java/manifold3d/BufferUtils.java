package manifold3d;

import org.bytedeco.javacpp.*;
import org.bytedeco.javacpp.annotation.*;

import manifold3d.pub.SimplePolygon;
import manifold3d.linalg.DoubleVec3Vector;
import manifold3d.linalg.DoubleVec4Vector;
import manifold3d.linalg.IntegerVec3Vector;
import manifold3d.linalg.IntegerVec4Vector;

@Platform(compiler = "cpp17", include= {"buffer_utils.hpp", "manifold/manifold.h"})
public class BufferUtils extends Pointer {
    static { Loader.load(); }

    public BufferUtils() { }

    public static native @ByVal UIntVector uIntVectorFromPointer(LongPointer values, @Cast("std::size_t") long count);
    public static native @ByVal IntVector intVectorFromPointer(IntPointer values, @Cast("std::size_t") long count);
    public static native @ByVal FloatVector floatVectorFromPointer(FloatPointer values, @Cast("std::size_t") long count);
    public static native @ByVal SimplePolygon createDoubleVec2Vector(DoublePointer values, @Cast("std::size_t") long count);
    public static native @ByVal DoubleVec3Vector createDoubleVec3Vector(DoublePointer values, @Cast("std::size_t") long count);
    public static native @ByVal IntegerVec3Vector createIntegerVec3Vector(IntPointer values, @Cast("std::size_t") long count);
    public static native @ByVal IntegerVec4Vector createIntegerVec4Vector(IntPointer values, @Cast("std::size_t") long count);
    public static native @ByVal DoubleVec4Vector createDoubleVec4Vector(DoublePointer values, @Cast("std::size_t") long count);
}
