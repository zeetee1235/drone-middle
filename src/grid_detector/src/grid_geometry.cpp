#include "grid_detector/grid_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>

namespace grid_detector {

namespace {

constexpr double kPi = 3.14159265358979323846;

double normalizeAngle180(double angle_deg) {
    double a = std::fmod(angle_deg, 180.0);
    if (a < 0.0) {
        a += 180.0;
    }
    return a;
}

double angularDistance(double a, double b) {
    const double diff = std::abs(normalizeAngle180(a) - normalizeAngle180(b));
    return std::min(diff, 180.0 - diff);
}

double minCoord(const ClassifiedLine& line) {
    if (line.orientation == Orientation::Horizontal) {
        return std::min(line.segment.start.x, line.segment.end.x);
    }
    return std::min(line.segment.start.y, line.segment.end.y);
}

double maxCoord(const ClassifiedLine& line) {
    if (line.orientation == Orientation::Horizontal) {
        return std::max(line.segment.start.x, line.segment.end.x);
    }
    return std::max(line.segment.start.y, line.segment.end.y);
}

ClassifiedLine makeMergedLine(
    const std::vector<ClassifiedLine>& group,
    Orientation orientation)
{
    const double weight_sum = std::accumulate(
        group.begin(), group.end(), 0.0,
        [](double acc, const ClassifiedLine& line) {
            return acc + std::max(1.0, line.length_px);
        });

    double axis = 0.0;
    double angle = 0.0;
    double length = 0.0;
    double min_span = std::numeric_limits<double>::infinity();
    double max_span = -std::numeric_limits<double>::infinity();

    for (const auto& line : group) {
        const double w = std::max(1.0, line.length_px);
        axis += line.axis_position_px * w;
        angle += line.angle_deg * w;
        length += line.length_px;
        min_span = std::min(min_span, minCoord(line));
        max_span = std::max(max_span, maxCoord(line));
    }
    axis /= weight_sum;
    angle /= weight_sum;

    ClassifiedLine merged;
    merged.orientation = orientation;
    merged.axis_position_px = axis;
    merged.angle_deg = angle;
    merged.length_px = length;

    if (orientation == Orientation::Horizontal) {
        merged.segment = {{min_span, axis}, {max_span, axis}};
    } else {
        merged.segment = {{axis, min_span}, {axis, max_span}};
    }
    return merged;
}

std::vector<ClassifiedLine> mergeOneOrientation(
    std::vector<ClassifiedLine> lines,
    Orientation orientation,
    const GridGeometryParams& params)
{
    std::sort(lines.begin(), lines.end(), [](const auto& a, const auto& b) {
        if (a.axis_position_px == b.axis_position_px) {
            return minCoord(a) < minCoord(b);
        }
        return a.axis_position_px < b.axis_position_px;
    });

    std::vector<ClassifiedLine> merged;
    std::vector<ClassifiedLine> group;

    for (const auto& line : lines) {
        if (line.orientation != orientation) {
            continue;
        }

        if (group.empty()) {
            group.push_back(line);
            continue;
        }

        const auto current = makeMergedLine(group, orientation);
        const bool same_axis = std::abs(line.axis_position_px - current.axis_position_px)
            <= params.merge_axis_tolerance_px;
        const bool connected_span = minCoord(line)
            <= maxCoord(current) + params.merge_gap_tolerance_px;

        if (same_axis && connected_span) {
            group.push_back(line);
        } else {
            merged.push_back(current);
            group.clear();
            group.push_back(line);
        }
    }

    if (!group.empty()) {
        merged.push_back(makeMergedLine(group, orientation));
    }
    return merged;
}

bool within(double value, double low, double high, double padding) {
    return value >= low - padding && value <= high + padding;
}

}  // namespace

double segmentLength(const LineSegment& line) {
    const double dx = line.end.x - line.start.x;
    const double dy = line.end.y - line.start.y;
    return std::sqrt(dx * dx + dy * dy);
}

double segmentAngleDeg(const LineSegment& line) {
    const double dx = line.end.x - line.start.x;
    const double dy = line.end.y - line.start.y;
    return normalizeAngle180(std::atan2(dy, dx) * 180.0 / kPi);
}

Orientation classifyOrientation(double angle_deg, double tolerance_deg) {
    if (tolerance_deg < 0.0) {
        throw std::invalid_argument("tolerance_deg must be non-negative");
    }
    const double angle = normalizeAngle180(angle_deg);
    if (angularDistance(angle, 0.0) <= tolerance_deg) {
        return Orientation::Horizontal;
    }
    if (angularDistance(angle, 90.0) <= tolerance_deg) {
        return Orientation::Vertical;
    }
    return Orientation::Other;
}

ClassifiedLine classifyLine(const LineSegment& line, const GridGeometryParams& params) {
    ClassifiedLine out;
    out.segment = line;
    out.angle_deg = segmentAngleDeg(line);
    out.length_px = segmentLength(line);
    out.orientation = classifyOrientation(out.angle_deg, params.angle_tolerance_deg);

    if (out.orientation == Orientation::Horizontal) {
        out.axis_position_px = (line.start.y + line.end.y) * 0.5;
    } else if (out.orientation == Orientation::Vertical) {
        out.axis_position_px = (line.start.x + line.end.x) * 0.5;
    }
    return out;
}

std::vector<ClassifiedLine> classifyLines(
    const std::vector<LineSegment>& lines,
    const GridGeometryParams& params)
{
    std::vector<ClassifiedLine> out;
    out.reserve(lines.size());
    for (const auto& line : lines) {
        auto classified = classifyLine(line, params);
        if (classified.orientation != Orientation::Other && classified.length_px > 1.0) {
            out.push_back(classified);
        }
    }
    return out;
}

std::vector<ClassifiedLine> mergeCollinearLines(
    const std::vector<ClassifiedLine>& lines,
    const GridGeometryParams& params)
{
    std::vector<ClassifiedLine> horizontal;
    std::vector<ClassifiedLine> vertical;
    horizontal.reserve(lines.size());
    vertical.reserve(lines.size());

    for (const auto& line : lines) {
        if (line.orientation == Orientation::Horizontal) {
            horizontal.push_back(line);
        } else if (line.orientation == Orientation::Vertical) {
            vertical.push_back(line);
        }
    }

    auto merged_h = mergeOneOrientation(horizontal, Orientation::Horizontal, params);
    auto merged_v = mergeOneOrientation(vertical, Orientation::Vertical, params);
    merged_h.insert(merged_h.end(), merged_v.begin(), merged_v.end());
    return merged_h;
}

std::vector<Point2> computeIntersections(
    const std::vector<ClassifiedLine>& lines,
    const GridGeometryParams& params)
{
    std::vector<const ClassifiedLine*> horizontal;
    std::vector<const ClassifiedLine*> vertical;

    for (const auto& line : lines) {
        if (line.orientation == Orientation::Horizontal) {
            horizontal.push_back(&line);
        } else if (line.orientation == Orientation::Vertical) {
            vertical.push_back(&line);
        }
    }

    std::vector<Point2> points;
    points.reserve(horizontal.size() * vertical.size());
    for (const auto* h : horizontal) {
        const double y = h->axis_position_px;
        const double h_min_x = std::min(h->segment.start.x, h->segment.end.x);
        const double h_max_x = std::max(h->segment.start.x, h->segment.end.x);

        for (const auto* v : vertical) {
            const double x = v->axis_position_px;
            const double v_min_y = std::min(v->segment.start.y, v->segment.end.y);
            const double v_max_y = std::max(v->segment.start.y, v->segment.end.y);

            if (within(x, h_min_x, h_max_x, params.intersection_padding_px)
                && within(y, v_min_y, v_max_y, params.intersection_padding_px)) {
                points.push_back({x, y});
            }
        }
    }
    return points;
}

GridGeometryResult analyzeGridGeometry(
    const std::vector<LineSegment>& raw_lines,
    const GridGeometryParams& params)
{
    GridGeometryResult out;
    const auto classified = classifyLines(raw_lines, params);
    out.merged_lines = mergeCollinearLines(classified, params);
    out.intersections = computeIntersections(out.merged_lines, params);

    double heading_weight_sum = 0.0;
    double heading = 0.0;
    for (const auto& line : out.merged_lines) {
        if (line.orientation == Orientation::Horizontal) {
            const double w = std::max(1.0, line.length_px);
            heading += line.angle_deg * w;
            heading_weight_sum += w;
        }
    }
    if (heading_weight_sum > 0.0) {
        out.dominant_heading_deg = heading / heading_weight_sum;
    }

    if (params.min_intersections_valid == 0) {
        out.confidence = out.intersections.empty() ? 0.0 : 1.0;
    } else {
        out.confidence = std::min(
            1.0,
            static_cast<double>(out.intersections.size())
                / static_cast<double>(params.min_intersections_valid));
    }
    return out;
}

}  // namespace grid_detector
