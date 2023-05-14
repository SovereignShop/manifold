package manifold3d;

import manifold3d.Glm.DoubleVec3;
import org.bytedeco.javacpp.*;
import org.bytedeco.javacpp.annotation.*;

@Platform(include = {"<vector>", "glm/glm.hpp"})
@Name("std::vector<glm::vec3>")
public class DoubleVec3Vector extends Pointer {
    static { Loader.load(); }

    public DoubleVec3Vector() { allocate(); }
    private native void allocate();

    public native @Cast("size_t") long size();
    public native void resize(@Cast("size_t") long n);

    @Name("operator[]") public native @ByRef DoubleVec3 get(@Cast("size_t") long i);
    @Name("push_back") public native void pushBack(@ByRef DoubleVec3 value);
}
