#include <gtest/gtest.h>
#include <pinocchio/algorithm/rnea.hpp>
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "roahm_dynamics/RNEA.hpp"

using namespace Roahm;
using namespace Roahm::Dynamics;

// Define a test fixture class
class YourClassTest : public ::testing::Test {
protected:
    // Set up the test fixture
    void SetUp() override {
        // Initialize any objects or variables needed for the tests
        std::srand(std::time(nullptr));

        const std::string model_path = "./models/urdf/gen3.urdf";
        modelPtr_ = std::make_shared<Model::model>(model_path);

        q = Eigen::VectorXd::Random(modelPtr_->NB);
        v = Eigen::VectorXd::Random(modelPtr_->NB);
        a = Eigen::VectorXd::Random(modelPtr_->NB);
    }

    // Tear down the test fixture
    void TearDown() override {
        // Clean up any resources used by the tests
    }

    std::shared_ptr<Model::model> modelPtr_;

    Eigen::VectorXd q, v, a;
};

TEST_F(YourClassTest, TestInitialAndFinalConditions) {
    // Arrange

    // Act
    q.resize(modelPtr_->NB);
    v.resize(modelPtr_->NB);
    a.resize(modelPtr_->NB);

    // inverse dynamics using pinocchio
    pinocchio::rnea(
        modelPtr_->model_pinocchio, 
        modelPtr_->data_pinocchio, 
        q, v, a);

    // // inverse dynamics using roahm_dynamics
    MultiBodyDynamics dynamics(modelPtr_);
    dynamics.rnea(q, v, v, a);

    std::cerr << modelPtr_->data_pinocchio.tau.transpose() << std::endl;
    std::cerr << dynamics.tau.transpose() << std::endl;

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