#include "visual_odometry/localization_core.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

namespace {

bool near(double a, double b, double eps = 1e-6) {
    return std::abs(a - b) <= eps;
}

void testMetersPerPixelAt2m() {
    const double mpp = visual_odometry::metersPerPixel(640, 2.0, 110.0);
    assert(mpp > 0.008);
    assert(mpp < 0.0095);
}

void testPixelFlowToVelocitySign() {
    visual_odometry::FlowMeasurement flow;
    flow.mean_dx_px = 10.0;
    flow.mean_dy_px = -5.0;
    flow.dt_sec = 0.1;
    flow.image_width_px = 640;
    flow.altitude_m = 2.0;

    const auto vel = visual_odometry::pixelFlowToVelocity(flow, 110.0);
    assert(vel.x < 0.0);
    assert(vel.y > 0.0);
}

void testIntegratePoseWithYaw() {
    visual_odometry::Pose2 pose{0.0, 0.0, M_PI / 2.0};
    visual_odometry::Vec2 body_velocity{1.0, 0.0};

    const auto next = visual_odometry::integratePose(pose, body_velocity, 2.0);
    assert(near(next.x, 0.0, 1e-6));
    assert(near(next.y, 2.0, 1e-6));
}

void testSnapPoseToGrid() {
    visual_odometry::Pose2 pose{2.82, 6.12, 0.1};
    const auto snap = visual_odometry::snapPoseToGrid(pose, 3.0, 0.4);
    assert(snap.snapped);
    assert(near(snap.pose.x, 3.0));
    assert(near(snap.pose.y, 6.0));
    assert(snap.correction_m > 0.0);
}

void testHomeGridTransformRoundTrip() {
    visual_odometry::Pose2 home_in_grid{6.0, 3.0, M_PI / 2.0};
    visual_odometry::Pose2 home_pose{2.0, 1.0, 0.2};

    const auto grid_pose = visual_odometry::transformHomeToGrid(home_pose, home_in_grid);
    const auto round_trip = visual_odometry::transformGridToHome(grid_pose, home_in_grid);

    assert(near(round_trip.x, home_pose.x));
    assert(near(round_trip.y, home_pose.y));
    assert(near(round_trip.yaw_rad, home_pose.yaw_rad));
}

}  // namespace

int main() {
    testMetersPerPixelAt2m();
    testPixelFlowToVelocitySign();
    testIntegratePoseWithYaw();
    testSnapPoseToGrid();
    testHomeGridTransformRoundTrip();
    std::cout << "visual_odometry_core_test passed\n";
    return 0;
}
