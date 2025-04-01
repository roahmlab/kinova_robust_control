#include <gtest/gtest.h>
#include "roahm_trajectories/Trajectories.hpp"

using namespace Roahm;

// Define a test fixture class
class BezierCurveTest : public ::testing::Test {
protected:
    // Set up the test fixture
    void SetUp() override {
        // Initialize any objects or variables needed for the tests
        std::srand(std::time(nullptr));
        double start_time = 0.0;
        double trajectory_duration = 2.345;
        double duration = trajectory_duration;
        int dof = 7;
        traj_data_.resize(dof * 6);

        q0 = Eigen::VectorXd::Random(dof);
        qd0 = Eigen::VectorXd::Random(dof);
        qdd0 = Eigen::VectorXd::Random(dof);
        q1 = Eigen::VectorXd::Random(dof);
        qd1 = Eigen::VectorXd::Random(dof);
        qdd1 = Eigen::VectorXd::Random(dof);

        for (int i = 0; i < dof; i++) {
            traj_data_[6 * i + 0] = q0(i);
            traj_data_[6 * i + 1] = qd0(i);
            traj_data_[6 * i + 2] = qdd0(i);
            traj_data_[6 * i + 3] = q1(i);
            traj_data_[6 * i + 4] = qd1(i);
            traj_data_[6 * i + 5] = qdd1(i);
        }

        trajPtr_ = std::make_shared<Trajectory>(
            start_time, 
            trajectory_duration, 
            duration, 
            dof,
            FIFTH_ORDER_BEZIER_TRAJ, 
            traj_data_.data());
    }

    // Tear down the test fixture
    void TearDown() override {
        // Clean up any resources used by the tests
    }

    std::vector<double> traj_data_;
    std::shared_ptr<Trajectory> trajPtr_;

    Eigen::VectorXd q0;
    Eigen::VectorXd qd0;
    Eigen::VectorXd qdd0;
    Eigen::VectorXd q1;
    Eigen::VectorXd qd1;
    Eigen::VectorXd qdd1;
};

TEST_F(BezierCurveTest, TestInitialAndFinalConditions) {
    // Arrange

    // Act
    TrajectoryData trajBegin = trajPtr_->compute(0.0);
    TrajectoryData trajEnd = trajPtr_->compute(trajPtr_->trajectory_duration);

    // Assert
    for (int i = 0; i < trajPtr_->dof; i++) {
        ASSERT_NEAR(q0(i), trajBegin.pos(i), 1e-10);
        ASSERT_NEAR(qd0(i), trajBegin.vel(i), 1e-10);
        ASSERT_NEAR(qdd0(i), trajBegin.acc(i), 1e-10);
        ASSERT_NEAR(q1(i), trajEnd.pos(i), 1e-10);
        ASSERT_NEAR(qd1(i), trajEnd.vel(i), 1e-10);
        ASSERT_NEAR(qdd1(i), trajEnd.acc(i), 1e-10);
    }
}

// Entry point for running the tests
int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}