#include <gtest/gtest.h>
#include "trajectories/Trajectories.hpp"

using namespace KinovaRobustControl;

// Define a test fixture class
class ArmourBezierCurveTest : public ::testing::Test {
protected:
    // Set up the test fixture
    void SetUp() override {
        // Initialize any objects or variables needed for the tests
        std::srand(std::time(nullptr));
        double start_time = 0.0;
        double trajectory_duration = 2.345;
        double duration = trajectory_duration;
        int dof = 7;
        traj_data_.resize(dof * 4);

        for (int i = 0; i < dof; i++) {
            traj_data_[4 * i + 0] = std::rand() % 66 - 33.0;
            traj_data_[4 * i + 1] = std::rand() % 66 - 33.0;
            traj_data_[4 * i + 2] = std::rand() % 66 - 33.0;
            traj_data_[4 * i + 3] = std::rand() % 66 - 33.0;
        }

        trajPtr_ = std::make_shared<Trajectory>(
            start_time, 
            trajectory_duration, 
            duration, 
            dof, 
            ARMOUR_TRAJ, 
            traj_data_.data());
    }

    // Tear down the test fixture
    void TearDown() override {
        // Clean up any resources used by the tests
    }

    std::vector<double> traj_data_;
    std::shared_ptr<Trajectory> trajPtr_;
};

TEST_F(ArmourBezierCurveTest, TestInitialAndFinalConditions) {
    // Arrange

    // Act
    TrajectoryData trajBegin = trajPtr_->compute(0.0);
    TrajectoryData trajEnd = trajPtr_->compute(trajPtr_->trajectory_duration);

    // Assert
    for (int i = 0; i < trajPtr_->dof; i++) {
        ASSERT_NEAR(traj_data_[4 * i + 0], trajBegin.pos(i), 1e-10);
        ASSERT_NEAR(traj_data_[4 * i + 1], trajBegin.vel(i), 1e-10);
        ASSERT_NEAR(traj_data_[4 * i + 2], trajBegin.acc(i), 1e-10);
        ASSERT_NEAR(traj_data_[4 * i + 3], trajEnd.pos(i), 1e-10);
        ASSERT_NEAR(0.0, trajEnd.vel(i), 1e-10);
        ASSERT_NEAR(0.0, trajEnd.acc(i), 1e-10);
    }
}

// Entry point for running the tests
int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}