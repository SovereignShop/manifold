package manifold3d.glm;

import org.bytedeco.javacpp.*;
import org.bytedeco.javacpp.annotation.*;

import java.util.Iterator;
import java.lang.Iterable;
import java.util.NoSuchElementException;

@Platform(compiler = "cpp17", include = "glm/glm.hpp")
@Namespace("glm")
@Name("vec2")
public class DoubleVec2 extends DoublePointer implements Iterable<Double> {
    static { Loader.load(); }

    @Override
    public Iterator<Double> iterator() {
        return new Iterator<Double>() {

            private int index = 0;

            @Override
            public boolean hasNext() {
                return index < 2;
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

    public DoubleVec2() { allocate(); }
    private native void allocate();

    public DoubleVec2(double x, double y) { allocate(x, y); }
    private native void allocate(double x, double y);

    @Name("operator []")
    public native double get(int i);

    public native @MemberGetter double x();
    public native @MemberGetter double y();

    public native @Name("operator=") @ByRef DoubleVec2 put(@ByRef DoubleVec2 rhs);
}
