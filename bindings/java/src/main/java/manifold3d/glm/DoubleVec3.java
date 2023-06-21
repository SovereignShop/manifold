package manifold3d.glm;

import org.bytedeco.javacpp.*;
import org.bytedeco.javacpp.annotation.*;

import java.util.Iterator;
import java.lang.Iterable;
import java.util.NoSuchElementException;

@Platform(compiler = "cpp17", include = "glm/glm.hpp")
@Namespace("glm")
@Name("vec3")
public class DoubleVec3 extends DoublePointer implements Iterable<Double> {
    static { Loader.load(); }

    @Override
    public Iterator<Double> iterator() {
        return new Iterator<Double>() {

            private int index = 0;

            @Override
            public boolean hasNext() {
                return index < 3;
            }

            @Override
            public Double next() {
                if (!hasNext()) {
                    throw new NoSuchElementException();
                }
                return get(index++);
            }
        };
    }

    public DoubleVec3() { allocate(); }
    private native void allocate();

    public DoubleVec3(double x, double y, double z) { allocate(x, y, z); }
    private native void allocate(double x, double y, double z);

    @Name("operator []")
    public native double get(int i);

    public native @MemberGetter double x();
    public native @MemberGetter double y();
    public native @MemberGetter double z();

    public native @Name("operator=") @ByRef DoubleVec3 put(@ByRef DoubleVec3 rhs);
}
