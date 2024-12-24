package manifold3d;

import org.junit.Test;
import manifold3d.Manifold;
import manifold3d.linalg.DoubleMat4x3;
import manifold3d.linalg.DoubleMat4x3Vector;
import manifold3d.pub.DoubleMesh;
import manifold3d.linalg.DoubleVec3;
import manifold3d.linalg.DoubleVec2;
import manifold3d.linalg.DoubleVec3Vector;
import manifold3d.MeshUtils;
import manifold3d.manifold.MeshIO;
import manifold3d.manifold.CrossSectionVector;
import manifold3d.manifold.CrossSection;
import manifold3d.manifold.ExportOptions;
import java.nio.DoubleBuffer;

public class ManifoldTest {

    public ManifoldTest() {}

    @Test
    public void testManifold() {
        // Existing test code
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

        // New test code: Creating a simple texture and exporting a GLB file
        // Define the dimensions of the height map
        int width = 10;
        int height = 10;
        float[] heightMap = new float[width * height];
        //Arrays.fill(heightMap, 1.0); // Flat surface

        double maxHeight = 20;

        // Generate a simple height map (e.g., a sine wave pattern)
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                // Create a sine wave pattern
                double z = Math.sin((double)x / width * 2 * Math.PI) * Math.sin((double)y / height * 2 * Math.PI) * maxHeight;
                heightMap[x + y * width] = (float)z;
            }
        }

        // Create a Manifold from the height map
        Manifold texturedSurface = MeshUtils.CreateSurface(heightMap, 1, width, height, 15.0);

        //System.out.println(texturedSurface.status());
        // Export the Manifold to a GLB file
        DoubleMesh texturedMesh = texturedSurface.getMesh();
        MeshIO.ExportMesh("TexturedSurface.glb", texturedMesh, opts);

        // Verify that the Manifold has a positive volume
        assert texturedSurface.getProperties().volume() > 0.0;
    }
}
