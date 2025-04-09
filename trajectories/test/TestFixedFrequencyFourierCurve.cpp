#include <gtest/gtest.h>
#include "trajectories/Trajectories.hpp"

using namespace KinovaRobustControl;

constexpr double traj_data[] = {
    0.232429,
    0.837561,
    0.978755,
    0.201056,
    -0.124087,
    -1.241260,
    0.598670,
    1.141100,
    -0.364967,
    0.936307,
    1.159270,
    0.008868,
    4.678150,
    0.343398,
    1.388960,
    1.047410,
    -3.506820,
    -0.433889,
    -1.213010,
    0.171025,
    0.041379,
    -0.401737,
    -0.239530,
    -0.680126,
    0.674508,
    2.153670,
    1.038250,
    2.148100,
    -0.316073,
    -0.263354,
    0.047878,
    -1.546970,
    -1.033540,
    0.050805,
    5.293520,
    0.681680,
    1.320040,
    0.620231,
    -5.065430,
    -1.334960,
    -2.955550,
    1.363410,
    2.087020,
    0.454433,
    0.008793,
    0.448008,
    -2.792450,
    -2.437650,
    -0.197305,
    0.662496,
    -0.257175,
    -1.041440,
    0.723770,
    0.839878,
    0.566712,
    0.008336,
    3.426860,
    1.989240,
    2.796310,
    4.476630,
    -2.301170,
    -1.365590,
    -1.410930,
    -1.837610,
    -0.430536,
    -0.029489,
    0.021637,
    -1.278840,
    0.869672,
    -1.056260,
    -3.419110,
    -4.648940,
    -2.683860,
    -0.882769,
    1.857120,
    0.816855,
    -2.401060,
    -0.153311,
    -0.298100,
    -24.509100,
    0.555528,
    -16.745800,
    -0.152563,
    8.868860,
    -1.224130,
    -0.080905,
    1.121050,
    -0.405652,
    -0.123332,
    -0.583831,
    -0.355939,
    2.0 * M_PI};

constexpr double q0[] = {-0.153311,-0.298100,0.623641,0.555528,2.103756,-0.152563,2.585675};
constexpr double qd0[] = {-1.224130,-0.080905,1.121050,-0.405652,-0.123332,-0.583831,-0.355939};
constexpr double qdd0[] = {2.107200,1.397520,1.571790,0.730409,-1.519910,2.088880,-7.028320};

constexpr double q1[] = {1.227250,0.426286,1.279741,0.596652,-2.959344,-0.310726,-2.677765};
constexpr double qd1[] = {1.100160,0.007776,-1.274250,0.102396,-0.035399,-0.500469,-0.139569};
constexpr double qdd1[] = {2.107200,1.397520,1.571790,0.730409,-1.519910,2.088880,-7.028320};

// Define a test fixture class
class ArmourBezierCurveTest : public ::testing::Test {
protected:
    // Set up the test fixture
    void SetUp() override {
        // Initialize any objects or variables needed for the tests
        double start_time = 0.0;
        double trajectory_duration = 10.0;
        double duration = trajectory_duration;
        int dof = 7;

        trajPtr_ = std::make_shared<Trajectory>(
            start_time, 
            trajectory_duration, 
            duration, 
            dof, 
            FOURIER_TRAJ, 
            traj_data);
    }

    // Tear down the test fixture
    void TearDown() override {
        // Clean up any resources used by the tests
    }

    std::shared_ptr<Trajectory> trajPtr_;
};

TEST_F(ArmourBezierCurveTest, TestInitialAndFinalConditions) {
    // Arrange

    // Act
    TrajectoryData trajBegin = trajPtr_->compute(0.0);
    TrajectoryData trajEnd = trajPtr_->compute(trajPtr_->trajectory_duration);

    // Assert
    for (int i = 0; i < trajPtr_->dof; i++) {
        ASSERT_NEAR(q0[i], trajBegin.pos(i), 1e-4);
        ASSERT_NEAR(qd0[i], trajBegin.vel(i), 1e-4);
        ASSERT_NEAR(qdd0[i], trajBegin.acc(i), 1e-4);
        ASSERT_NEAR(q1[i], trajEnd.pos(i), 1e-4);
        ASSERT_NEAR(qd1[i], trajEnd.vel(i), 1e-4);
        ASSERT_NEAR(qdd1[i], trajEnd.acc(i), 1e-4);
    }
}

// Entry point for running the tests
int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}