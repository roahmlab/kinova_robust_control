#include "kortex/KortexBlock.hpp"
#include "kortex/KortexRobot.hpp"
#include "system/System.hpp"
#include <gtest/gtest.h>
#include <stdexcept>

using namespace KinovaRobustControl::Kortex;

class ZeroControlBlock : public KinovaRobustControl::System::BaseBlock
{
  private:
    using Base = KinovaRobustControl::System::BaseBlock;

  public:
    static Base::SharedPtr make_shared()
    {
        return Base::SharedPtr(new ZeroControlBlock());
    }

    virtual void run() override
    {
        ControlMsg control(7);
        default_output->set(control);

        // get value and update
        while (true)
        {
            check_stop();
            auto measure_ptr = measurement->get_new<Measurement>();
            control.frame_id = measure_ptr->frame_id + 1;
            default_output->set(control);
        }
    }

  private:
    ZeroControlBlock()
        : KinovaRobustControl::System::BaseBlock(
              "zero_block",
              KinovaRobustControl::System::OutputPort<ControlMsg>::make_shared()),
          measurement(register_input<Measurement>("measurement"))
    {
    }

    KinovaRobustControl::System::AbstractInputPort::SharedPtr measurement;
};

// TEST(KortexTest, KortexRobotTest)
// {
//     /** Ctor */
//     KortexSettings settings;
//     KortexRobot kr(settings);
//     // constructor, should fail
//     // KortexRobot kr;
//     // KortexRobot copy(kr);

//     /** Init */
//     ASSERT_THROW(kr.MoveHome(), std::runtime_error);
//     ASSERT_THROW(kr.MoveZero(), std::runtime_error);
//     ASSERT_THROW(kr.MovePredefined("Home"), std::runtime_error);
//     ASSERT_THROW(kr.MoveJointPos(std::vector<float>{0, 0, 0, 0, 0, 0, 0}),
//                  std::runtime_error);
//     ASSERT_TRUE(kr.Init());

//     /** High Level Commands */
//     ASSERT_TRUE(kr.MoveHome());
//     ASSERT_TRUE(kr.MoveZero());
//     ASSERT_TRUE(kr.MovePredefined("Retract"));

//     ASSERT_TRUE(kr.MoveJointPos(std::vector<float>{0, 0, 0, 0, 0, 0, 0}));
// }

TEST(KortexTest, KortexBlockTest)
{
    KortexSettings settings;
    settings.JOINT_ENB = 0b1111111;

    KinovaRobustControl::System::System system("KortexBlockTestSystem");

    auto zero_control = ZeroControlBlock::make_shared();
    auto kortex_block = KortexBlock::make_shared("kortex_block", settings);

    // register block
    system.connect_blocks(kortex_block, zero_control)
        .connect_blocks(zero_control, kortex_block)
        .set_timeout(10)
        .set_verbose();

    // run block
    system.run();
}

int main(int argc, char *argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
