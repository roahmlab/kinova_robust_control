#pragma once
#include <queue>
#include <deque>

namespace KinovaRobustControl
{
namespace Filter
{
/**
 * @brief running average filter
 **/
template <typename T> class RunningAverageFilter
{
  public:
    /**
     * @brief constructor
     * @param num_samples: samples used for averaging
     **/
    RunningAverageFilter(const std::size_t num_samples)
        : sum(0), num_samples(num_samples)
    {
    }

    RunningAverageFilter() = delete;

    /**
     * @brief filter given input using new update and previous values
     **/
    T filter(const T &input)
    {
        samples.push(input);
        sum += input;

        // need to remove element
        if (samples.size() > num_samples)
        {
            sum -= samples.front();
            samples.pop();
        }
        return sum / samples.size();
    }

  private:
    std::queue<T> samples;
    T sum;
    std::size_t num_samples;
};

/**
 * @brief BiquadFilter
 * @details expression:
 * y[n] = b0 * x[n] + b1 * x[n-1] + b2 * x[n-2] + a1 * y[n-1] + a2y[n-2];
 * */
template <typename T> class BiquadFilter
{
  public:
    BiquadFilter(double b0, double b1, double b2, double a0, double a1)
        : a0(a0), a1(a1), b0(b0), b1(b1), b2(b2)
    {
    }

    BiquadFilter() = delete;

    /**
     * @brief perform filtering on input
     * @param input: input to pass in
     **/
    T filter(const T &input)
    {
        // first few
        if (w.size() < 2)
        {
            w.push_front(input);
            return input;
        }

        // w[n] = x[n] - a0w[n-1] - a1w[n-2]
        w.push_front(input - a0 * w[0] - a1 * w[1]);

        // y[n] = b0w[n] + b1w[n-1] + b2w[n-2]
        T y = b0 * w[0] + b1 * w[1] + b2 * w[2];

        // remove oldest w
        w.pop_back();
        return y;
    }

  private:
    const double a0, a1, b0, b1, b2;
    std::deque<T> w;
};
} // namespace Utils
} // namespace KinovaRobustControl
