#include "mot3d/geometry/box_overlap.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

namespace mot3d {
namespace {

// A point on the ground: x east, y north, in metres.
struct Point {
  double x;
  double y;
};

// Corners in order, going counter-clockwise.
using Polygon = std::vector<Point>;

// The box's 4 ground corners, counter-clockwise. Length runs along the heading,
// width across it.
Polygon Footprint(const OrientedBox& box) {
  const double half_length = box.size[1] / 2.0;
  const double half_width = box.size[0] / 2.0;
  const double c = std::cos(box.yaw);
  const double s = std::sin(box.yaw);
  const Point local[4] = {{half_length, half_width},
                          {-half_length, half_width},
                          {-half_length, -half_width},
                          {half_length, -half_width}};
  Polygon corners;
  for (const Point& p : local) {
    corners.push_back({box.center.x() + p.x * c - p.y * s, box.center.y() + p.x * s + p.y * c});
  }
  return corners;
}

// Positive if p is left of the line from a to b, negative if right, 0 if on it.
double Side(const Point& a, const Point& b, const Point& p) {
  return (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
}

// Where the edge from p to q crosses the line through a and b.
Point Crossing(const Point& p, const Point& q, const Point& a, const Point& b) {
  const double side_p = Side(a, b, p);
  const double side_q = Side(a, b, q);
  const double t = side_p / (side_p - side_q);
  return {p.x + t * (q.x - p.x), p.y + t * (q.y - p.y)};
}

// The part of `shape` that lies inside `window`. Both must be counter-clockwise,
// and `window` must be convex (a rectangle is). Trims `shape` by one edge of
// `window` at a time, keeping only what is on the inner (left) side.
Polygon Clip(Polygon shape, const Polygon& window) {
  for (std::size_t i = 0; i < window.size() && !shape.empty(); ++i) {
    const Point& a = window[i];
    const Point& b = window[(i + 1) % window.size()];
    Polygon kept;
    for (std::size_t j = 0; j < shape.size(); ++j) {
      const Point& prev = shape[(j + shape.size() - 1) % shape.size()];
      const Point& cur = shape[j];
      const bool prev_inside = Side(a, b, prev) >= 0.0;
      const bool cur_inside = Side(a, b, cur) >= 0.0;
      if (prev_inside != cur_inside) {
        kept.push_back(Crossing(prev, cur, a, b));
      }
      if (cur_inside) {
        kept.push_back(cur);
      }
    }
    shape = std::move(kept);
  }
  return shape;
}

// Area of a polygon from its corners (the "shoelace" formula).
double Area(const Polygon& shape) {
  double twice_area = 0.0;
  for (std::size_t i = 0; i < shape.size(); ++i) {
    const Point& p = shape[i];
    const Point& q = shape[(i + 1) % shape.size()];
    twice_area += p.x * q.y - q.x * p.y;
  }
  return std::abs(twice_area) / 2.0;
}

// How much the two boxes' height ranges overlap, in metres (0 if they don't).
double HeightOverlap(const OrientedBox& a, const OrientedBox& b) {
  const double top = std::min(a.center.z() + a.size[2] / 2.0, b.center.z() + b.size[2] / 2.0);
  const double bottom = std::max(a.center.z() - a.size[2] / 2.0, b.center.z() - b.size[2] / 2.0);
  return std::max(0.0, top - bottom);
}

double Volume(const OrientedBox& box) {
  return box.size[0] * box.size[1] * box.size[2];
}

}  // namespace

double BoxIou(const OrientedBox& a, const OrientedBox& b) {
  const double overlap = Area(Clip(Footprint(a), Footprint(b))) * HeightOverlap(a, b);
  const double total = Volume(a) + Volume(b) - overlap;
  return total > 0.0 ? overlap / total : 0.0;
}

}  // namespace mot3d