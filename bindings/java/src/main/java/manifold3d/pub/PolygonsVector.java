package manifold3d.pub;

import manifold3d.pub.Polygons;
import org.bytedeco.javacpp.*;
import org.bytedeco.javacpp.annotation.*;

import java.util.Iterator;
import java.lang.Iterable;
import java.util.NoSuchElementException;

@Platform(compiler = "cpp17", include = "manifold/manifold.h")
@Name("std::vector<manifold::Polygons>")
public class PolygonsVector extends Pointer implements Iterable<Polygons> {
    static { Loader.load(); }

    @Override
    public Iterator<Polygons> iterator() {
        return new Iterator<Polygons>() {

            private long index = 0;

            @Override
            public boolean hasNext() {
                return index < size();
            }

            @Override
            public Polygons next() {
                if (!hasNext()) {
                    throw new NoSuchElementException();
                }
                return get(index++);
            }
        };
    }

    public PolygonsVector() { allocate(); }
    private native void allocate();

    public native @Cast("size_t") long size();
    public native void resize(@Cast("size_t") long n);

    @Name("operator[]") public native @ByRef Polygons get(@Cast("size_t") long i);
    @Name("push_back") public native void pushBack(@ByRef Polygons value);
}
