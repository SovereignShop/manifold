package manifold3d;


import java.nio.IntBuffer;

import manifold3d.BufferUtils;

import org.bytedeco.javacpp.*;
import org.bytedeco.javacpp.annotation.*;

@Platform(compiler = "cpp17", include = "<vector>")
@Name("std::vector<int>")
public class IntVector extends Pointer {
    static { Loader.load(); }


    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public IntVector(IntPointer p) { super(p); }
    public IntVector() { allocate(); }
    public static IntVector FromArray(int[] intArray) {
        IntPointer intPtr = new IntPointer(intArray);
        return BufferUtils.intVectorFromPointer(intPtr, intArray.length);
    }
    public static IntVector FromBuffer(IntBuffer intBuffer) {
        IntPointer intPtr = new IntPointer(intBuffer);
        return new IntVector(intPtr);
    }
    private native void allocate();
    public native IntPointer data();
    public native @Cast("size_t") long size();
    public native @Cast("bool") boolean empty();
    public native void resize(@Cast("size_t") long n);
    public native void reserve(@Cast("size_t") long n);
    public native @Name("operator[]") int get(@Cast("size_t") long n);
    public native @Name("push_back") void pushBack(int value);

    public IntBuffer asIntBuffer() {
        return new IntPointer(this).asBuffer();
    }

    public int[] toIntArray() {
        // Allocate an array of the appropriate size
        int size = (int) size();
        int[] result = new int[size];
        IntPointer ptr = data();
        ptr.get(result);
        return result;
    }
}
