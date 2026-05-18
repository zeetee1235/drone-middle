#pragma once

#include <cstddef>
#include <vector>

namespace grid_detector {

struct Point2 {
    double x = 0.0;
    double y = 0.0;
};

struct LineSegment {
    Point2 start;
    Point2 end;
};

enum class Orientation {
    Horizontal,
    Vertical,
    Other,
};

struct ClassifiedLine {
    LineSegment segment;
    Orientation orientation = Orientation::Other;
    double angle_deg = 0.0;
    double axis_position_px = 0.0;
    double length_px = 0.0;
};

struct GridGeometryParams {
    double angle_tolerance_deg = 10.0;
    double merge_axis_tolerance_px = 8.0;
    double merge_gap_tolerance_px = 20.0;
    double intersection_padding_px = 12.0;
    std::size_t min_intersections_valid = 2;
};

struct GridGeometryResult {
    std::vector<ClassifiedLine> merged_lines;
    std::vector<Point2> intersections;
    double dominant_heading_deg = 0.0;
    double confidence = 0.0;
};

double segmentLength(const LineSegment& line);
double segmentAngleDeg(const LineSegment& line);
Orientation classifyOrientation(double angle_deg, double tolerance_deg);
ClassifiedLine classifyLine(const LineSegment& line, const GridGeometryParams& params);

std::vector<ClassifiedLine> classifyLines(
    const std::vector<LineSegment>& lines,
    const GridGeometryParams& params);

std::vector<ClassifiedLine> mergeCollinearLines(
    const std::vector<ClassifiedLine>& lines,
    const GridGeometryParams& params);

std::vector<Point2> computeIntersections(
    const std::vector<ClassifiedLine>& lines,
    const GridGeometryParams& params);

GridGeometryResult analyzeGridGeometry(
    const std::vector<LineSegment>& raw_lines,
    const GridGeometryParams& params = {});

}  // namespace grid_detector
