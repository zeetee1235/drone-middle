#include "grid_detector/grid_geometry.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

namespace {

bool near(double a, double b, double eps = 1e-6) {
    return std::abs(a - b) <= eps;
}

void testOrientationClassification() {
    grid_detector::GridGeometryParams params;
    assert(grid_detector::classifyOrientation(2.0, params.angle_tolerance_deg)
        == grid_detector::Orientation::Horizontal);
    assert(grid_detector::classifyOrientation(178.0, params.angle_tolerance_deg)
        == grid_detector::Orientation::Horizontal);
    assert(grid_detector::classifyOrientation(91.0, params.angle_tolerance_deg)
        == grid_detector::Orientation::Vertical);
    assert(grid_detector::classifyOrientation(45.0, params.angle_tolerance_deg)
        == grid_detector::Orientation::Other);
}

void testMergeSplitLines() {
    grid_detector::GridGeometryParams params;
    params.merge_axis_tolerance_px = 5.0;
    params.merge_gap_tolerance_px = 15.0;

    const std::vector<grid_detector::LineSegment> raw = {
        {{0.0, 10.0}, {50.0, 11.0}},
        {{55.0, 9.0}, {100.0, 10.0}},
        {{20.0, 0.0}, {19.0, 100.0}},
    };

    const auto result = grid_detector::analyzeGridGeometry(raw, params);
    assert(result.merged_lines.size() == 2);
    assert(result.intersections.size() == 1);
    assert(near(result.intersections[0].x, 19.5, 1.0));
    assert(near(result.intersections[0].y, 10.0, 1.0));
}

void testIntersectionsForGrid() {
    grid_detector::GridGeometryParams params;
    params.min_intersections_valid = 4;

    const std::vector<grid_detector::LineSegment> raw = {
        {{0.0, 100.0}, {300.0, 100.0}},
        {{0.0, 200.0}, {300.0, 200.0}},
        {{100.0, 0.0}, {100.0, 300.0}},
        {{200.0, 0.0}, {200.0, 300.0}},
    };

    const auto result = grid_detector::analyzeGridGeometry(raw, params);
    assert(result.merged_lines.size() == 4);
    assert(result.intersections.size() == 4);
    assert(near(result.confidence, 1.0));
}

}  // namespace

int main() {
    testOrientationClassification();
    testMergeSplitLines();
    testIntersectionsForGrid();
    std::cout << "grid_detector_core_test passed\n";
    return 0;
}
