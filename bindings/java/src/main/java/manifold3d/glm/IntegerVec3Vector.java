package manifold3d.glm;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.IntBuffer;

import manifold3d.BufferUtils;
import manifold3d.glm.IntegerVec3;
import org.bytedeco.javacpp.*;
import org.bytedeco.javacpp.annotation.*;

@Platform(include = {"<vector>", "glm/glm.hpp"})
@Name("std::vector<glm::ivec3>")
public class IntegerVec3Vector extends Pointer {
    static { Loader.load(); }

    public IntegerVec3Vector() { allocate(); }
    private native void allocate();

    public native @Cast("size_t") long size();
    public native void resize(@Cast("size_t") long n);

    @Name("operator[]") public native @ByRef IntegerVec3 get(@Cast("size_t") long i);
    @Name("push_back") public native void pushBack(@ByRef IntegerVec3 value);

    public static IntegerVec3Vector FromBuffer(IntBuffer buff) {
        IntPointer ptr = new IntPointer(buff);
        return BufferUtils.createIntegerVec3Vector(ptr, buff.capacity());
    }

    public static IntegerVec3Vector FromArray(int[] data) {
        ByteBuffer buff = ByteBuffer.allocateDirect(data.length * Integer.BYTES).order(ByteOrder.nativeOrder());
        IntBuffer intBuffer = buff.asIntBuffer();
        intBuffer.put(data);
        intBuffer.flip();

        return FromBuffer(intBuffer);
    }

}