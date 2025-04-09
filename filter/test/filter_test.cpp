#include "filter/Filters.hpp"
#include <gtest/gtest.h>
using namespace KinovaRobustControl::Filter;

TEST(FilterTest, RunningAverageFilter)
{
    RunningAverageFilter<double> filter(5);
    for (int i = 0; i < 10; i++)
    {
        std::cout << filter.filter(i) << ", ";
    }
    std::cout << std::endl;
}

TEST(FilterTest, BiquadFilter)
{
    BiquadFilter<double> filter(0.7, 0.2, 0.1, 0.1, 0.1);
    for (int i = 0; i < 30; i++)
    {
        std::cout << filter.filter(i) << ", ";
    }
    std::cout << std::endl;
}

int main(int argc, char *argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
