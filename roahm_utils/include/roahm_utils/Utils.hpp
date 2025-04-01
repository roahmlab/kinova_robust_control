#pragma once
#include <eigen3/Eigen/Dense>
#include <mutex>
#include <vector>
#include <cmath>
#include <fstream>

namespace Roahm
{
namespace Utils
{

/**
 * @brief convert degree to radian
 * @param deg: angle in degree
 **/
double deg2rad(const double deg);

/**
 * @brief convert radian to degree
 * @param rad: angle in radian
 **/
double rad2deg(const double rad);

/**
 * @brief wrap angle to [-pi, pi]
 * @param angle: angle in radian
 **/
double wrapToPi(const double angle);

/**
 * @brief wrap an Eigen vector of angles to [-pi, pi]
 * @param angles: vector of angles in radian
 **/
Eigen::VectorXd wrapToPi(const Eigen::VectorXd& angles);

/**
 * @brief get sign of a number
 * @param val: input number
 * @param eps: tolerance for zero
 **/
double sign(double val, double eps = 1e-8);

/**
 * @brief reshape a vector to matrix
 * @param vec: input vector
 * @param rows: number of rows
 * @param cols: number of columns
 **/
Eigen::MatrixXd reshape(const Eigen::VectorXd& vec, int rows, int cols);

/** 
 * @brief convert std::vector to Eigen vector
 * @param vec: input vector
 **/
Eigen::VectorXd convertVectorToEigen(const std::vector<double>& vec);

/**
 * @brief construct Eigen diagonal matrix fromm std::vector
 * @param diagonal: vector of elements on diagonal
 **/
Eigen::MatrixXd DiagonalMatrixXd(const std::vector<double>& diagonal);

Eigen::MatrixXd initializeEigenMatrixFromFile(std::ifstream& inputfile);

/**
 * @brief check if two Eigen vectors are equal
 * @param a: first vector
 * @param b: second vector
 * @param tol: tolerance for comparison
 * @param debug: print debug message
 **/
bool ifTwoVectorEqual(
    const Eigen::VectorXd& a, 
    const Eigen::VectorXd& b, 
    const double tol = 1e-10,
    const bool debug = false);

/**
 * @brief convert SRC type to eigen matrix
 * @details need both SRC type to have data() method
 **/
template <typename SRC, typename DST>
DST stl2Vec(const SRC &src, const size_t size = 0)
{
    size_t l = size;
    if (l == 0)
        l = src.size();
    DST dst;
    if (l != dst.size())
    {
        dst.resize(l);
    }
    std::copy(src.data(), src.data() + l, dst.data());
    return dst;
}

/**
 * @brief disable copy constructor and copy assignment operator for derived
 *class
 * @details usage: inherit from this class and declare it as private
 * @ref
 *https://www.boost.org/doc/libs/1_73_0/libs/core/doc/html/core/noncopyable.html
 **/
class noncopyable
{
  public:
    noncopyable() = default;
    ~noncopyable() = default;
    noncopyable(const noncopyable &) = delete;
    noncopyable &operator=(const noncopyable &) = delete;
};

/**
 * @brief stop signal used in thread
 **/
class thread_stop : public std::exception
{
  public:
    virtual const char *what() const noexcept override
    {
        return "Thread signaled to stop";
    }
};

namespace Spatial 
{
/**
 * @brief skew symmetric matrix from vector
 * @param v: 3x1 Eigen vector
 **/
Eigen::Matrix3d skew(const Eigen::Vector3d& v);

/**
 * @brief skew symmetric matrix from vector
 * @param m: 3x3 Eigen matrix
 **/
Eigen::Vector3d skew(const Eigen::Matrix3d& m);

/**
 * @brief unskew vector from skew symmetric matrix
 * @param m: 3x3 Eigen matrix
 **/
Eigen::Vector3d unskew(const Eigen::Matrix3d& m);

/**
 * @brief compose Plucker coordinate transformation matrix
 * @param R: 3x3 rotation matrix
 * @param p: 3x1 vector
 **/
Eigen::Matrix<double, 6, 6> plux(const Eigen::Matrix3d& R, const Eigen::Vector3d& p);
} // namespace Spatial

namespace Signal
{
enum class Signal
{
    Interrupt = 2,
};

class SignalException : public std::runtime_error
{
  public:
    template <Signal SIG> static SignalException make()
    {
        return SignalException(SIG);
    }

    SignalException(Signal signal)
        : std::runtime_error("Signal Exception"), signal(signal)
    {
    }

    /**
     * @brief get signal type
     **/
    inline Signal what_signal()
    {
        return signal;
    }

  private:
    Signal signal;
};

[[noreturn]] inline void InterruptSignalHandler([[maybe_unused]] int _ignored)
{
    throw SignalException::make<Signal::Interrupt>();
}

/**
 * @brief Set up the signal handlers for certain signal
 */
void setupSignalHandlers(Signal sig = Signal::Interrupt,
                         void (*handler)(int) = InterruptSignalHandler);
} // namespace Signal
} // namespace Utils
} // namespace Roahm
