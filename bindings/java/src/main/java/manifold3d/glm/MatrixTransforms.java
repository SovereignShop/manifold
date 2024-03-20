package manifold3d.glm;

import manifold3d.glm.DoubleMat4x3;
import manifold3d.glm.DoubleMat3x2;
import manifold3d.glm.DoubleVec3;
import manifold3d.glm.DoubleVec2;

import org.bytedeco.javacpp.*;
import org.bytedeco.javacpp.annotation.*;

@Platform(compiler = "cpp17", include = {"matrix_transforms.hpp"})
public class MatrixTransforms extends Pointer {
    static { Loader.load(); }

    public static native @ByVal DoubleMat4x3 Yaw(@ByRef DoubleMat4x3 mat, double angle);
    public static native @ByVal DoubleMat4x3 Pitch(@ByRef DoubleMat4x3 mat, double angle);
    public static native @ByVal DoubleMat4x3 Roll(@ByRef DoubleMat4x3 mat, double angle);

    public static native @ByVal DoubleMat4x3 Rotate(@ByRef DoubleMat4x3 mat, @ByRef DoubleVec3 angles);
    public static native @ByVal DoubleMat4x3 Rotate(@ByRef DoubleMat4x3 mat, @ByRef DoubleVec3 axis, double angle);
    public static native @ByVal DoubleMat3x2 Rotate(@ByRef DoubleMat3x2 mat, double angleRadians);
    public static native @ByVal DoubleMat4x3 Translate(@ByRef DoubleMat4x3 mat, @ByRef DoubleVec3 vec);
    public static native @ByVal DoubleMat3x2 Translate(@ByRef DoubleMat3x2 mat, @ByRef DoubleVec2 vec);
    public static native @ByVal DoubleMat4x3 SetTranslation(@ByRef DoubleMat4x3 mat, @ByRef DoubleVec3 vec);
    public static native @ByVal DoubleMat3x2 SetTranslation(@ByRef DoubleMat3x2 mat, @ByRef DoubleVec2 vec);

    public static native @ByVal DoubleMat4x3 Transform(@ByRef DoubleMat4x3 mat1, @ByRef DoubleMat4x3 mat2);
    public static native @ByVal DoubleMat4x3 InvertTransform(@ByRef DoubleMat4x3 mat);
    public static native @ByVal DoubleMat3x2 InvertTransform(@ByRef DoubleMat3x2 mat);
    public static native @ByVal DoubleMat4x3 CombineTransforms(@ByRef DoubleMat4x3 a, @ByRef DoubleMat4x3 b);
    public static native @ByVal DoubleMat3x2 CombineTransforms(@ByRef DoubleMat3x2 a, @ByRef DoubleMat3x2 b);
}
