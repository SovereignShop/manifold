package manifold3d.linalg;

import java.nio.DoubleBuffer;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.DoubleBuffer;

import java.util.Arrays;

import manifold3d.BufferUtils;
import manifold3d.linalg.DoubleMat4x3;
import org.bytedeco.javacpp.*;
import org.bytedeco.javacpp.annotation.*;

import java.util.ArrayList;
import java.util.Iterator;
import java.lang.Iterable;
import java.util.NoSuchElementException;

@Platform(compiler = "cpp17", include = {"<vector>", "linalg.h"})
@Name("std::vector<linalg::mat<double, 4, 3>>")
public class DoubleMat4x3Vector extends Pointer implements Iterable<DoubleMat4x3> {
    static { Loader.load(); }

    @Override
    public Iterator<DoubleMat4x3> iterator() {
        return new Iterator<DoubleMat4x3>() {

            private long index = 0;

            @Override
            public boolean hasNext() {
                return index < size();
            }

            @Override
            public DoubleMat4x3 next() {
                if (!hasNext()) {
                    throw new NoSuchElementException();
                }
                return get(index++);
            }
        };
    }
    public DoubleMat4x3Vector(ArrayList<DoubleMat4x3> mats) {
        allocate();
        for (DoubleMat4x3 section: mats) {
            this.pushBack(section);
        }
    }

    public DoubleMat4x3Vector(DoubleMat4x3... mats) {
        allocate();
        for (DoubleMat4x3 section : mats) {
            this.pushBack(section);
        }
    }

    public DoubleMat4x3Vector() { allocate(); }
    private native void allocate();

    public native @Cast("size_t") long size();
    public native void resize(@Cast("size_t") long n);

    @Name("operator[]") public native @ByRef DoubleMat4x3 get(@Cast("size_t") long i);
    @Name("push_back") public native void pushBack(@ByRef DoubleMat4x3 value);
}
