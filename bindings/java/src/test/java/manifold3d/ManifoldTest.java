package manifold3d;

import org.junit.Test;
import org.junit.Assert;
import manifold3d.Manifold;
import manifold3d.glm.DoubleMat4x3;
import manifold3d.glm.DoubleMat4x3Vector;
import manifold3d.pub.DoubleMesh;
import manifold3d.glm.DoubleVec3;
import manifold3d.glm.DoubleVec2;
import manifold3d.glm.DoubleVec3Vector;
import manifold3d.MeshUtils;
import manifold3d.manifold.MeshIO;
import manifold3d.manifold.CrossSectionVector;
import manifold3d.manifold.CrossSection;
import manifold3d.manifold.ExportOptions;

public class ManifoldTest {

    public ManifoldTest() {}

    @Test
    public void testManifold() {
        DoubleMesh mesh = new DoubleMesh();
        Manifold manifold = new Manifold(mesh);

        Manifold sphere = Manifold.Sphere(10.0f, 20);
        Manifold cube = Manifold.Cube(new DoubleVec3(15.0f, 15.0f, 15.0f), false);
        Manifold cylinder = Manifold.Cylinder(3, 30.0f, 30.0f, 0, false).translateX(20).translateY(20).translateZ(-3.0);

        Manifold diff = cube.subtract(sphere);
        Manifold intersection = cube.intersect(sphere);
        Manifold union = cube.add(sphere);

        DoubleMesh diffMesh = diff.getMesh();
        DoubleMesh intersectMesh = intersection.getMesh();
        DoubleMesh unionMesh = union.getMesh();
        ExportOptions opts = new ExportOptions();
        opts.faceted(false);

        MeshIO.ExportMesh("CubeMinusSphere.stl", diffMesh, opts);
        MeshIO.ExportMesh("CubeIntersectSphere.glb", intersectMesh, opts);
        MeshIO.ExportMesh("CubeUnionSphere.obj", unionMesh, opts);

        Manifold hull = cylinder.convexHull(cube.translateZ(100.0));
        DoubleMesh hullMesh = hull.getMesh();

        MeshIO.ExportMesh("hull.glb", hullMesh, opts);
        assert hull.getProperties().volume() > 0.0;

        DoubleMat4x3 frame1 = new DoubleMat4x3(1).translate(new DoubleVec3(20, 0, 0));
        DoubleMat4x3 frame2 = new DoubleMat4x3(1)
                .rotate(new DoubleVec3(0, -3.14, 0))
                .translate(new DoubleVec3(20, 0, 0));
        CrossSection section1 = CrossSection.Square(new DoubleVec2(20, 20), true);
        CrossSection section2 = CrossSection.Circle(15, 20);
        Manifold loft = MeshUtils.Loft(new CrossSectionVector(section1, section2),
                                       new DoubleMat4x3Vector(frame1, frame2),
                                       MeshUtils.LoftAlgorithm.EagerNearestNeighbor);

        assert loft.getProperties().volume() > 0.0;

        DoubleVec3Vector vertPos = hullMesh.vertPos();
    }
}
