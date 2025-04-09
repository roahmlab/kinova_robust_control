#include "utils/Utils.hpp"
#include <cstddef>
#include <errno.h>
#include <iostream>
#include <signal.h>
#include <stdexcept>

namespace KinovaRobustControl
{
namespace Utils
{

double deg2rad(const double deg) 
{
    return deg * M_PI / 180.0;
}

double rad2deg(const double rad) 
{
    return rad * 180.0 / M_PI;
}

double wrapToPi(const double angle) 
{
    double res = angle;
    while (res > M_PI) {
        res -= 2.0 * M_PI;
    }
    while (res < -M_PI) {
        res += 2.0 * M_PI;
    }
    return res;
}

Eigen::VectorXd wrapToPi(const Eigen::VectorXd& angles) 
{
    Eigen::VectorXd res = angles;
    for (int i = 0; i < res.size(); i++) {
        res(i) = wrapToPi(res(i));
    }
    return res;
}

double sign(double val, double eps) 
{
    if (val > eps) {
        return 1.0;
    } 
    else if (val < -eps) {
        return -1.0;
    } 
    else {
        return 0.0;
    }
}

Eigen::MatrixXd reshape(const Eigen::VectorXd& vec, int rows, int cols) {
    if (vec.size() != rows * cols) {
        throw std::invalid_argument("reshape: input vector size does not match the size of the matrix");
    }

    return Eigen::Map<const Eigen::MatrixXd>(vec.data(), rows, cols);
}

Eigen::VectorXd convertVectorToEigen(const std::vector<double>& vec)
{
    Eigen::VectorXd res(vec.size());
    for (size_t i = 0; i < vec.size(); i++) {
        res(i) = vec[i];
    }
    return res;
}

Eigen::MatrixXd DiagonalMatrixXd(const std::vector<double>& diagonal)
{
    Eigen::MatrixXd res = Eigen::MatrixXd::Zero(diagonal.size(), diagonal.size());
    for (size_t i = 0; i < diagonal.size(); i++) {
        res(i, i) = diagonal[i];
    }
    return res;
}

bool ifTwoVectorEqual(
    const Eigen::VectorXd& a, 
    const Eigen::VectorXd& b, 
    const double tol,
    const bool debug) {
    if (a.size() != b.size()) {
        return false;
    }
    for (int i = 0; i < a.size(); i++) {
        if (fabs(a(i) - b(i)) > tol) {
            if (debug) {
                std::cerr << "ifTwoVectorEqual: " 
                          << "a(" << i << ") = " << a(i) << ", "
                          << "b(" << i << ") = " << b(i) << ", " 
                          << "with difference: " << fabs(a(i) - b(i)) << std::endl;
            }

            return false;
        }
    }
    return true;
}

Eigen::MatrixXd initializeEigenMatrixFromFile(std::ifstream& inputfile) {
    if (!inputfile.is_open()) {
        throw std::runtime_error("Cannot open input file!");
    }

    std::vector<std::vector<double>> data;
    std::string line;

    while (std::getline(inputfile, line)) {
        std::istringstream iss(line);
        std::vector<double> lineData;
        double value;
        while (iss >> value) {
            lineData.push_back(value);
        }
        data.push_back(lineData);
    }

    if (data.size() == 0) {
        return Eigen::MatrixXd(0, 0);
    }

    Eigen::MatrixXd res(data.size(), data[0].size());
    for (size_t i = 0; i < data.size(); i++) {
        for (size_t j = 0; j < data[0].size(); j++) {
            res(i, j) = data[i][j];
        }
    }

    return res;
}

// namespace Spatial {
// Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
//     Eigen::Matrix3d res;
//     res << 0,    -v(2), v(1),
//            v(2),  0,   -v(0),
//           -v(1),  v(0), 0;
//     return res;
// }

// Eigen::Vector3d skew(const Eigen::Matrix3d& m) 
// {
//     Eigen::Vector3d res;
//     res << m(2,1) - m(1,2), 
//            m(0,2) - m(2,0), 
//            m(1,0) - m(0,1);
//     return 0.5 * res;
// }

// Eigen::Vector3d unskew(const Eigen::Matrix3d& m) 
// {
//     Eigen::Vector3d res;
//     res << m(2,1), 
//            m(0,2), 
//            m(1,0);
//     return res;
// }

// Eigen::Matrix<double, 6, 6> plux(const Eigen::Matrix3d& R, const Eigen::Vector3d& p) 
// {
//     Eigen::Matrix<double, 6, 6> res;
//     res << R,            Eigen::MatrixXd::Zero(3, 3),
//            -R * skew(p), R;
//     return res;
// }
// } // namespace Spatial

namespace Signal
{
void setupSignalHandlers(Signal sig, void (*handler)(int))
{
    if (signal((int)sig, handler) == SIG_ERR)
    {
        throw std::runtime_error("Error setting up signal handlers");
    }
}
} // namespace Signal
} // namespace Utils
} // namespace KinovaRobustControl
