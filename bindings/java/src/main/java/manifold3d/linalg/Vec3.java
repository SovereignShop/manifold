package manifold3d.linalg;

import org.bytedeco.javacpp.*;
import org.bytedeco.javacpp.annotation.*;

import java.util.Iterator;
import java.lang.Iterable;
import java.util.NoSuchElementException;

@Platform(compiler = "cpp17", include = "linalg.h")
@Namespace("linalg")
@Name("vec<double, 3>")
public class Vec3 extends DoublePointer implements Iterable<Double> {
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

    public Vec3() { allocate(); }
    private native void allocate();

    public Vec3(double x, double y, double z) { allocate(x, y, z); }
    private native void allocate(double x, double y, double z);

    @Name("operator []")
    public native double get(int i);

    public double x() {
        return get(0);
    }
    public double y() {
        return get(1);
    }
    public double z() {
        return get(2);
    }

    public native @Name("operator=") @ByRef Vec3 put(@ByRef Vec3 rhs);
}
