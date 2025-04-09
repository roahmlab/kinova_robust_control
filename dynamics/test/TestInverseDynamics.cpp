#include <gtest/gtest.h>
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/regressor.hpp"
#include "dynamics/RNEA.hpp"

using namespace KinovaRobustControl;
using namespace KinovaRobustControl::Dynamics;

// Define a test fixture class
class YourClassTest : public ::testing::Test {
protected:
    // Set up the test fixture
    void SetUp() override {
        // Initialize any objects or variables needed for the tests
        std::srand(std::time(nullptr));

        const std::string model_path = "./models/urdf/gen3.urdf";
        modelPtr_ = std::make_shared<Model::model>(model_path);

        // disable motor dynamics in this test
        modelPtr_->friction.setZero();
        modelPtr_->damping.setZero();
        modelPtr_->transmissionInertia.setZero();
        modelPtr_->offset.setZero();
        modelPtr_->model_pinocchio.friction.setZero();
        modelPtr_->model_pinocchio.damping.setZero();
        modelPtr_->model_pinocchio.armature.setZero();
    }

    // Tear down the test fixture
    void TearDown() override {
        // Clean up any resources used by the tests
    }

    std::shared_ptr<Model::model> modelPtr_;

    Eigen::VectorXd q, v, a;
};

TEST_F(YourClassTest, TestInverseDynamics) {
    // Arrange

    // Act
    q = Eigen::VectorXd::Random(modelPtr_->NB);
    v = Eigen::VectorXd::Random(modelPtr_->NB);
    a = Eigen::VectorXd::Random(modelPtr_->NB);

    // inverse dynamics using pinocchio
    pinocchio::rnea(
        modelPtr_->model_pinocchio, 
        modelPtr_->data_pinocchio, 
        q, v, a);

    // inverse dynamics using dynamics
    MultiBodyDynamics dynamics(modelPtr_);
    dynamics.rnea(q, v, v, a);

    // Assert
    for (int i = 0; i < modelPtr_->NB; i++) {
        ASSERT_NEAR(modelPtr_->data_pinocchio.tau(i), dynamics.tau(i), 1e-8);
    }
}

// Entry point for running the tests
int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}