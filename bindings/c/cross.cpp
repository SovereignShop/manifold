#include <conv.h>
#include <manifold.h>
#include <public.h>

#include "cross_section.h"
#include "types.h"

ManifoldCrossSection *manifold_cross_section_empty(void *mem) {
  return to_c(new (mem) CrossSection());
}

ManifoldCrossSection *manifold_cross_section_copy(void *mem,
                                                  ManifoldCrossSection *cs) {
  auto cross = *from_c(cs);
  return to_c(new (mem) CrossSection(cross));
}

ManifoldCrossSection *manifold_cross_section_of_simple_polygon(
    void *mem, ManifoldSimplePolygon *p, ManifoldFillRule fr) {
  return to_c(new (mem) CrossSection(*from_c(p), from_c(fr)));
}

ManifoldCrossSection *manifold_cross_section_of_polygons(void *mem,
                                                         ManifoldPolygons *p,
                                                         ManifoldFillRule fr) {
  return to_c(new (mem) CrossSection(*from_c(p), from_c(fr)));
}

ManifoldCrossSection *manifold_cross_section_square(void *mem, float x, float y,
                                                    int center) {
  auto cs = CrossSection::Square(glm::vec2(x, y), center);
  return to_c(new (mem) CrossSection(cs));
}

ManifoldCrossSection *manifold_cross_section_circle(void *mem, float radius,
                                                    int circular_segments) {
  auto cs = CrossSection::Circle(radius, circular_segments);
  return to_c(new (mem) CrossSection(cs));
}

ManifoldCrossSection *manifold_cross_section_union(void *mem,
                                                   ManifoldCrossSection *a,
                                                   ManifoldCrossSection *b) {
  auto cs = (*from_c(a)) + (*from_c(b));
  return to_c(new (mem) CrossSection(cs));
}

ManifoldCrossSection *manifold_cross_section_difference(
    void *mem, ManifoldCrossSection *a, ManifoldCrossSection *b) {
  auto cs = (*from_c(a)) - (*from_c(b));
  return to_c(new (mem) CrossSection(cs));
}

ManifoldCrossSection *manifold_cross_section_intersection(
    void *mem, ManifoldCrossSection *a, ManifoldCrossSection *b) {
  auto cs = (*from_c(a)) ^ (*from_c(b));
  return to_c(new (mem) CrossSection(cs));
}

ManifoldCrossSection *manifold_cross_section_rect_clip(void *mem,
                                                       ManifoldCrossSection *cs,
                                                       ManifoldRect *r) {
  auto clipped = from_c(cs)->RectClip(*from_c(r));
  return to_c(new (mem) CrossSection(clipped));
}

ManifoldCrossSection *manifold_cross_section_translate(void *mem,
                                                       ManifoldCrossSection *cs,
                                                       float x, float y) {
  auto translated = from_c(cs)->Translate(glm::vec2(x, y));
  return to_c(new (mem) CrossSection(translated));
}

ManifoldCrossSection *manifold_cross_section_rotate(void *mem,
                                                    ManifoldCrossSection *cs,
                                                    float deg) {
  auto rotated = from_c(cs)->Rotate(deg);
  return to_c(new (mem) CrossSection(rotated));
}

ManifoldCrossSection *manifold_cross_section_scale(void *mem,
                                                   ManifoldCrossSection *cs,
                                                   float x, float y) {
  auto scaled = from_c(cs)->Scale(glm::vec2(x, y));
  return to_c(new (mem) CrossSection(scaled));
}

ManifoldCrossSection *manifold_cross_section_mirror(void *mem,
                                                    ManifoldCrossSection *cs,
                                                    float ax_x, float ax_y) {
  auto mirrored = from_c(cs)->Mirror(glm::vec2(ax_x, ax_y));
  return to_c(new (mem) CrossSection(mirrored));
}

ManifoldCrossSection *manifold_cross_section_transform(void *mem,
                                                       ManifoldCrossSection *cs,
                                                       float x1, float y1,
                                                       float x2, float y2,
                                                       float x3, float y3) {
  auto mat = glm::mat3x2(x1, y1, x2, y2, x3, y3);
  auto transformed = from_c(cs)->Transform(mat);
  return to_c(new (mem) CrossSection(transformed));
}

ManifoldCrossSection *manifold_cross_section_simplify(void *mem,
                                                      ManifoldCrossSection *cs,
                                                      double epsilon) {
  auto simplified = from_c(cs)->Simplify(epsilon);
  return to_c(new (mem) CrossSection(simplified));
}

ManifoldCrossSection *manifold_cross_section_offset(
    void *mem, ManifoldCrossSection *cs, double delta, ManifoldJoinType jt,
    double miter_limit, double arc_tolerance) {
  auto offset =
      from_c(cs)->Offset(delta, from_c(jt), miter_limit, arc_tolerance);
  return to_c(new (mem) CrossSection(offset));
}

double manifold_cross_section_area(ManifoldCrossSection *cs) {
  return from_c(cs)->Area();
}

int manifold_cross_section_is_empty(ManifoldCrossSection *cs) {
  return from_c(cs)->IsEmpty();
}

ManifoldRect *manifold_cross_section_bounds(void *mem,
                                            ManifoldCrossSection *cs) {
  auto rect = from_c(cs)->Bounds();
  return to_c(new (mem) Rect(rect));
}

ManifoldPolygons *manifold_cross_section_to_polygons(void *mem,
                                                     ManifoldCrossSection *cs) {
  auto ps = from_c(cs)->ToPolygons();
  return to_c(new (mem) Polygons(ps));
}
