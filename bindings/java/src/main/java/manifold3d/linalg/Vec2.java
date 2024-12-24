package manifold3d.linalg;

import org.bytedeco.javacpp.*;
import org.bytedeco.javacpp.annotation.*;

import java.util.Iterator;
import java.lang.Iterable;
import java.util.NoSuchElementException;

@Platform(compiler = "cpp17", include = "linalg.h")
@Namespace("linalg")
@Name("vec2")
public class Vec2 extends DoublePointer implements Iterable<Double> {
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

    public Vec2() { allocate(); }
    private native void allocate();

    public Vec2(double x, double y) { allocate(x, y); }
    private native void allocate(double x, double y);

    @Name("operator []")
    public native double get(int i);

    public double x() {
        return get(0);
    }
    public double y() {
        return get(1);
    }

    public native @Name("operator=") @ByRef Vec2 put(@ByRef Vec2 rhs);
}
