package manifold3d;

import org.junit.Test;
import manifold3d.Manifold;
import manifold3d.glm.DoubleVec2;
import manifold3d.pub.DoubleMesh;
import manifold3d.pub.SimplePolygon;
import manifold3d.pub.Polygons;
import manifold3d.manifold.MeshIO;
import manifold3d.manifold.ExportOptions;
import manifold3d.manifold.CrossSection;
import manifold3d.manifold.CrossSection.FillRule;
import manifold3d.manifold.MeshIO;
import manifold3d.manifold.ExportOptions;
import manifold3d.Manifold;

public class CrossSectionTest {

    public CrossSectionTest() {}

    @Test
    public void testCrossSection() {
        SimplePolygon polygon = new SimplePolygon();
        polygon.pushBack(new DoubleVec2(-10.0, -10.0));
        polygon.pushBack(new DoubleVec2(10.0, -10.0));
        polygon.pushBack(new DoubleVec2(10.0, 10.0));
        polygon.pushBack(new DoubleVec2(-10.0, 10.0));

        SimplePolygon innerPolygon = new SimplePolygon();
        innerPolygon.pushBack(new DoubleVec2(-5.0, -5.0));
        innerPolygon.pushBack(new DoubleVec2(5.0, -5.0));
        innerPolygon.pushBack(new DoubleVec2(5.0, 5.0));
        innerPolygon.pushBack(new DoubleVec2(-5.0, 5.0));

        CrossSection section = new CrossSection(polygon, FillRule.NonZero.ordinal());
        CrossSection innerSection = new CrossSection(innerPolygon, FillRule.NonZero.ordinal())
            .translate(new DoubleVec2(3, 0));

        CrossSection text = CrossSection.Text("DejaVuSans.ttf", "abcdefghijk lmnopqrstuvwxyz", 48, 6, FillRule.NonZero.ordinal());

        CrossSection circle = CrossSection.Circle(3.0f, 20);
        Manifold cylinder = Manifold.Extrude(circle, 50, 60, 0, new DoubleVec2(1.0, 1.0));

        CrossSection unionSection = section.convexHull(CrossSection.Circle(5, 0).translateX(60));

        assert unionSection.area() > 0.0;

        Manifold man = Manifold.Extrude(unionSection, 50, 60, 0, new DoubleVec2(1.0, 1.0));
        DoubleMesh mesh = man.getMesh();
        ExportOptions opts = new ExportOptions();
        MeshIO.ExportMesh("CrossSectionTest.stl", mesh, opts);
        MeshIO.ExportMesh("TextExtrusion.stl", Manifold.Extrude(text, 200, 1, 0, new DoubleVec2(1.0, 1.0)).getMesh(), opts);
    }
}
