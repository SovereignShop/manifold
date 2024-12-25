package manifold3d.linalg;

import manifold3d.linalg.DoubleMat3x4;
import manifold3d.linalg.DoubleMat2x3;
import manifold3d.linalg.DoubleVec3;
import manifold3d.linalg.DoubleVec2;

import org.bytedeco.javacpp.*;
import org.bytedeco.javacpp.annotation.*;

@Platform(compiler = "cpp17", include = {"matrix_transforms.hpp"})
public class MatrixTransforms extends Pointer {
    static { Loader.load(); }

    public static native @ByVal DoubleMat3x4 Yaw(@ByRef DoubleMat3x4 mat, double angle);
    public static native @ByVal DoubleMat3x4 Pitch(@ByRef DoubleMat3x4 mat, double angle);
    public static native @ByVal DoubleMat3x4 Roll(@ByRef DoubleMat3x4 mat, double angle);

    public static native @ByVal DoubleMat3x4 Rotate(@ByRef DoubleMat3x4 mat, @ByRef DoubleVec3 angles);
    public static native @ByVal DoubleMat3x4 Rotate(@ByRef DoubleMat3x4 mat, @ByRef DoubleVec3 axis, double angle);
    public static native @ByVal DoubleMat2x3 Rotate(@ByRef DoubleMat2x3 mat, double angleRadians);
    public static native @ByVal DoubleMat3x4 Translate(@ByRef DoubleMat3x4 mat, @ByRef DoubleVec3 vec);
    public static native @ByVal DoubleMat2x3 Translate(@ByRef DoubleMat2x3 mat, @ByRef DoubleVec2 vec);
    public static native @ByVal DoubleMat3x4 SetTranslation(@ByRef DoubleMat3x4 mat, @ByRef DoubleVec3 vec);
    public static native @ByVal DoubleMat2x3 SetTranslation(@ByRef DoubleMat2x3 mat, @ByRef DoubleVec2 vec);

    public static native @ByVal DoubleMat3x4 Transform(@ByRef DoubleMat3x4 mat1, @ByRef DoubleMat3x4 mat2);
    public static native @ByVal DoubleMat3x4 InvertTransform(@ByRef DoubleMat3x4 mat);
    public static native @ByVal DoubleMat2x3 InvertTransform(@ByRef DoubleMat2x3 mat);
    public static native @ByVal DoubleMat3x4 CombineTransforms(@ByRef DoubleMat3x4 a, @ByRef DoubleMat3x4 b);
    public static native @ByVal DoubleMat2x3 CombineTransforms(@ByRef DoubleMat2x3 a, @ByRef DoubleMat2x3 b);
}
