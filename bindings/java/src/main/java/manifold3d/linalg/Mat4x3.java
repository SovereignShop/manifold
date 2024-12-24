package manifold3d.linalg;

import manifold3d.linalg.Vec3;
import manifold3d.linalg.MatrixTransforms;

import org.bytedeco.javacpp.*;
import org.bytedeco.javacpp.annotation.*;

import java.util.Iterator;
import java.lang.Iterable;
import java.util.NoSuchElementException;

@Platform(compiler = "cpp17", include = "linalg.h")
@Namespace("linalg")
@Name("mat<double, 4, 3>")
public class Mat4x3 extends DoublePointer implements Iterable<Vec3> {
    static { Loader.load(); }

    @Override
    public Iterator<Vec3> iterator() {
        return new Iterator<Vec3>() {

            private int index = 0;

            @Override
            public boolean hasNext() {
                return index < 4;
            }

            @Override
            public Vec3 next() {
                if (!hasNext()) {
                    throw new NoSuchElementException();
                }
                return getColumn(index++);
            }
        };
    }

    public Mat4x3() { allocate(); }
    private native void allocate();

    private native void allocate(double x);
    public Mat4x3(double x) { allocate(x); }

    public Mat4x3(@ByRef Vec3 col1, @ByRef Vec3 col2,
                        @ByRef Vec3 col3, @ByRef Vec3 col4) {
        allocate(col1, col2, col3, col4);
    }
    public native void allocate(@ByRef Vec3 col1, @ByRef Vec3 col2,
                                @ByRef Vec3 col3, @ByRef Vec3 col4);

    public Mat4x3(double c0, double c1, double c2, double c3,
                  double c4, double c5, double c6, double c7,
                  double c8, double c9, double c10, double c11) {
        allocate(c0, c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11);
    }
    public native void allocate(double c0, double c1, double c2, double c3,
                                double c4, double c5, double c6, double c7,
                                double c8, double c9, double c10, double c11);

    @Name("operator[]") public native @ByRef Vec3 getColumn(int i);

    public native @Name("operator=") @ByRef Mat4x3 put(@ByRef Mat4x3 rhs);

    public Mat4x3 transform(@ByRef Mat4x3 other) {
        return MatrixTransforms.Transform(this, other);
    }

    public Mat4x3 rotate(@ByRef Vec3 angles) {
        return MatrixTransforms.Rotate(this, angles);
    }

    public Mat4x3 rotate(@ByRef Vec3 axis, float angle) {
        return MatrixTransforms.Rotate(this, axis, angle);
    }

    public Mat4x3 translate(@ByRef Vec3 vec) {
        return MatrixTransforms.Translate(this, vec);
    }
    public Mat4x3 translate(double x, double y, double z) {
        return MatrixTransforms.Translate(this, new Vec3(x, y, z));
    }
}
