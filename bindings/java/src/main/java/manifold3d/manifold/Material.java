package manifold3d.manifold;

import org.bytedeco.javacpp.*;
import org.bytedeco.javacpp.annotation.*;

import manifold3d.linalg.DoubleVec3;

@Platform(compiler = "cpp17", include = "manifold/meshIO.h")
@Namespace("manifold")
public class Material extends Pointer {
    static { Loader.load(); }

    public Material() { allocate(); }
    public native void allocate();

    public native @MemberGetter float roughness();
    public native @MemberSetter void roughness(float roughness);

    public native @MemberGetter float metalness();
    public native @MemberSetter void metalness(float metalness);

    public native @MemberGetter @ByRef DoubleVec3 color();
    public native @MemberSetter void color(@ByRef DoubleVec3 color);

    public native @MemberGetter int alpha();
    public native @MemberSetter void alpha(int alpha);

    public native @MemberGetter int normalIdx();
    public native @MemberSetter void normalIdx(int normalIdx);

    public native @MemberGetter int colorIdx();
    public native @MemberSetter void colorIdx(int colorIdx);

    public native @MemberGetter int alphaIdx();
    public native @MemberSetter void alphaIdx(int colorIdx);
}
