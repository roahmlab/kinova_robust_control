#include <unistd.h>

#include "roahm_trajectories/TrajectoryManager.hpp"

using namespace Roahm;

int main() {
    std::srand(std::time(nullptr));

    const uint32_t dof = 7;
    const Eigen::MatrixXd wayPoints = Eigen::VectorXd::Random(5, dof);

    double traj_data1[4 * dof];
    for (uint32_t i = 0; i < dof; i++) {
        traj_data1[4 * i + 0] = wayPoints(0, i);
        traj_data1[4 * i + 1] = 0;
        traj_data1[4 * i + 2] = 0;
        traj_data1[4 * i + 3] = wayPoints(1, i);
    }
    double traj_data2[4 * dof];
    for (uint32_t i = 0; i < dof; i++) {
        traj_data2[4 * i + 0] = wayPoints(1, i);
        traj_data2[4 * i + 1] = 0;
        traj_data2[4 * i + 2] = 0;
        traj_data2[4 * i + 3] = wayPoints(2, i);
    }
    double traj_data3[4 * dof];
    for (uint32_t i = 0; i < dof; i++) {
        traj_data3[4 * i + 0] = wayPoints(2, i);
        traj_data3[4 * i + 1] = 0;
        traj_data3[4 * i + 2] = 0;
        traj_data3[4 * i + 3] = wayPoints(3, i);
    }
    double traj_data4[4 * dof];
    for (uint32_t i = 0; i < dof; i++) {
        traj_data4[4 * i + 0] = wayPoints(3, i);
        traj_data4[4 * i + 1] = 0;
        traj_data4[4 * i + 2] = 0;
        traj_data4[4 * i + 3] = wayPoints(4, i);
    } 

    TrajectoryManager tm;

    std::shared_ptr<Trajectory> traj1 = 
        std::make_shared<Trajectory>(0.0, 1.0, 1.0, dof, ARMOUR_TRAJ, traj_data1);
    std::shared_ptr<Trajectory> traj2 =
        std::make_shared<Trajectory>(1.0, 1.0, 1.0, dof, ARMOUR_TRAJ, traj_data2);
    std::shared_ptr<Trajectory> traj3 =
        std::make_shared<Trajectory>(2.0, 1.0, 1.0, dof, ARMOUR_TRAJ, traj_data3);
    std::shared_ptr<Trajectory> traj4 =
        std::make_shared<Trajectory>(3.0, 1.0, 1.0, dof, ARMOUR_TRAJ, traj_data4);

    // Test 1: this is a feasible sequence
    std::cout << "Test 1 started: " << std::endl;
    try {
        const auto now = std::chrono::system_clock::now();
        const double t_rel = 
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                now.time_since_epoch()).count()
                    / 1.0e9;

        traj1->start_time = t_rel + 1e-2;
        tm.add_trajectory(traj1);

        usleep(4e5);
        traj2->start_time = t_rel + 0.8;
        tm.add_trajectory(traj2);

        usleep(8e5);
        traj3->start_time = t_rel + 1.6;
        tm.add_trajectory(traj3);

        usleep(1.2e6);
        traj4->start_time = t_rel + 2.4;
        tm.add_trajectory(traj4);
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
    }
    tm.reset();
    std::cout << "Test 1 ended!" << std::endl;
    std::cout << "============================" << std::endl;

    // Test 2: this is a NOT feasible sequence
    std::cout << "Test 2 started: " << std::endl;
    try {
        const auto now = std::chrono::system_clock::now();
        const double t_rel = 
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                now.time_since_epoch()).count()
                    / 1.0e9;

        traj1->start_time = t_rel + 1e-2;
        tm.add_trajectory(traj1);

        usleep(1.2e6);
        traj1->start_time = t_rel + 1.0;
        tm.add_trajectory(traj2);

        usleep(1e6);
        traj1->start_time = t_rel + 2.0;
        tm.add_trajectory(traj3);

        usleep(1e6);
        traj1->start_time = t_rel + 3.0;
        tm.add_trajectory(traj4);
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
    }
    tm.reset();
    std::cout << "Test 2 ended!" << std::endl;
    std::cout << "============================" << std::endl;

    // Test 3: this is a NOT feasible sequence
    std::cout << "Test 3 started: " << std::endl;
    try {
        const auto now = std::chrono::system_clock::now();
        const double t_rel = 
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                now.time_since_epoch()).count()
                    / 1.0e9;

        traj1->start_time = t_rel + 1e-2;
        tm.add_trajectory(traj1);

        traj2->start_time = t_rel + 1.0;
        tm.add_trajectory(traj2);

        traj3->start_time = t_rel + 1.0;
        tm.add_trajectory(traj3);

        traj4->start_time = t_rel + 1.0;
        tm.add_trajectory(traj4);
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
    }
    tm.reset();
    std::cout << "Test 3 ended!" << std::endl;
    std::cout << "============================" << std::endl;

    // Test 4: this is a NOT feasible sequence
    std::cout << "Test 4 started: " << std::endl;
    traj2->start_time = 0.5;
    traj3->start_time = 1.4;
    try {
        const auto now = std::chrono::system_clock::now();
        const double t_rel = 
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                now.time_since_epoch()).count()
                    / 1.0e9;

        traj1->start_time = t_rel + 1e-2;
        tm.add_trajectory(traj1);

        traj2->start_time = t_rel + 0.5;
        tm.add_trajectory(traj2);

        traj3->start_time = t_rel + 1.4;
        tm.add_trajectory(traj3);

        traj4->start_time = t_rel + 10.0;
        tm.add_trajectory(traj4);
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
    }
    tm.reset();
    std::cout << "Test 4 ended!" << std::endl;
    std::cout << "============================" << std::endl;

    // Test 5: this is a NOT feasible sequence
    std::cout << "Test 5 started: " << std::endl;
    traj1->duration = 0.5;
    traj2->start_time = 0.5;
    traj2->duration = 0.5;
    traj3->start_time = 1.0;
    traj3->duration = 0.5;
    traj4->start_time = 1.5;
    try {
        const auto now = std::chrono::system_clock::now();
        const double t_rel = 
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                now.time_since_epoch()).count()
                    / 1.0e9;

        traj1->start_time = t_rel + 1e-2;
        traj1->duration = 0.5;      
        tm.add_trajectory(traj1);

        traj2->start_time = traj1->start_time + traj1->duration;
        traj2->duration = 0.5;   
        tm.add_trajectory(traj2);

        traj3->start_time = traj2->start_time + traj2->duration;
        traj3->duration = 0.5;
        tm.add_trajectory(traj3);

        traj4->start_time = traj3->start_time + traj3->duration;
        traj4->duration = 0.5;
        tm.add_trajectory(traj4);
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
    }
    tm.reset();
    std::cout << "Test 5 ended!" << std::endl;
    std::cout << "============================" << std::endl;

    // Test 6: this is a feasible sequence
    std::cout << "Test 6 started: " << std::endl;
    auto trajData = traj1->compute(traj1->start_time + traj1->duration);
    for (uint32_t i = 0; i < dof; i++) {
        traj2->traj_data[4 * i + 0] = trajData.pos(i);
        traj2->traj_data[4 * i + 1] = trajData.vel(i);
        traj2->traj_data[4 * i + 2] = trajData.acc(i);
    }
    trajData = traj2->compute(traj2->start_time + traj2->duration);
    for (uint32_t i = 0; i < dof; i++) {
        traj3->traj_data[4 * i + 0] = trajData.pos(i);
        traj3->traj_data[4 * i + 1] = trajData.vel(i);
        traj3->traj_data[4 * i + 2] = trajData.acc(i);
    }
    trajData = traj3->compute(traj3->start_time + traj3->duration);
    for (uint32_t i = 0; i < dof; i++) {
        traj4->traj_data[4 * i + 0] = trajData.pos(i);
        traj4->traj_data[4 * i + 1] = trajData.vel(i);
        traj4->traj_data[4 * i + 2] = trajData.acc(i);
    }

    const auto start_clock = std::chrono::system_clock::now();
    try {
        const auto now = std::chrono::system_clock::now();
        const double t_rel = 
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                now.time_since_epoch()).count()
                    / 1.0e9;

        traj1->start_time = t_rel + 1e-2;
        tm.add_trajectory(traj1);

        traj2->start_time = traj1->start_time + traj1->duration;
        tm.add_trajectory(traj2);

        traj3->start_time = traj2->start_time + traj2->duration;
        tm.add_trajectory(traj3);

        traj4->start_time = traj3->start_time + traj3->duration;
        tm.add_trajectory(traj4);
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
    }
    // tm.reset();
    std::cout << "Test 6 ended!" << std::endl;
    std::cout << "============================" << std::endl;

    // Test 7: this is a feasible sequence
    std::cout << "Test 7 started: " << std::endl;
    while(!tm.is_empty()) {
        auto trajData = tm.get_desired(std::chrono::system_clock::now());
        // std::cout << "pos: " << trajData.pos.transpose() << std::endl;
        // std::cout << "vel: " << trajData.vel.transpose() << std::endl;
        // std::cout << "acc: " << trajData.acc.transpose() << std::endl;
        // std::cout << "============================" << std::endl;
    }
    auto time_taken = std::chrono::system_clock::now() - start_clock;
    tm.reset();
    std::cout << "Time taken: " << std::chrono::duration_cast<std::chrono::milliseconds>(time_taken).count() / 1000.0 << " s" << std::endl;

    std::cout << "Test 7 ended!" << std::endl;
    std::cout << "============================" << std::endl;

    // Test 8: this is a feasible sequence, with piecewise-continuous Bezier curves
    std::cout << "Test 8 started: " << std::endl;
    traj1->trajectory_type = FIFTH_ORDER_BEZIER_TRAJ;
    traj1->trajectory_duration = 1.0;
    traj1->duration = 1.0;
    for (uint32_t i = 0; i < dof; i++) { 
        traj1->traj_data[6 * i + 0] = 0.0;
        traj1->traj_data[6 * i + 1] = 0.0;
        traj1->traj_data[6 * i + 2] = 0.0;
        traj1->traj_data[6 * i + 3] = 0.1;
        traj1->traj_data[6 * i + 4] = 0.2;
        traj1->traj_data[6 * i + 5] = 0.3;
    }
    traj2->trajectory_type = FIFTH_ORDER_BEZIER_TRAJ;
    traj2->trajectory_duration = 1.5;
    traj2->duration = 1.5;
    for (uint32_t i = 0; i < dof; i++) { 
        traj2->traj_data[6 * i + 0] = 0.1;
        traj2->traj_data[6 * i + 1] = 0.2;
        traj2->traj_data[6 * i + 2] = 0.3;
        traj2->traj_data[6 * i + 3] = 0.4;
        traj2->traj_data[6 * i + 4] = 0.5;
        traj2->traj_data[6 * i + 5] = 0.6;
    }
    traj3->trajectory_type = FIFTH_ORDER_BEZIER_TRAJ;
    traj3->trajectory_duration = 2.0;
    traj3->duration = 2.0;
    for (uint32_t i = 0; i < dof; i++) { 
        traj3->traj_data[6 * i + 0] = 0.4;
        traj3->traj_data[6 * i + 1] = 0.5;
        traj3->traj_data[6 * i + 2] = 0.6;
        traj3->traj_data[6 * i + 3] = 0.7;
        traj3->traj_data[6 * i + 4] = 0.8;
        traj3->traj_data[6 * i + 5] = 0.9;
    }
    traj4->trajectory_type = FIFTH_ORDER_BEZIER_TRAJ;
    traj4->trajectory_duration = 2.5;
    traj4->duration = 2.5;
    for (uint32_t i = 0; i < dof; i++) { 
        traj4->traj_data[6 * i + 0] = 0.7;
        traj4->traj_data[6 * i + 1] = 0.8;
        traj4->traj_data[6 * i + 2] = 0.9;
        traj4->traj_data[6 * i + 3] = 1.0;
        traj4->traj_data[6 * i + 4] = 1.1;
        traj4->traj_data[6 * i + 5] = 1.2;
    }

    try {
        const auto now = std::chrono::system_clock::now();
        const double t_rel = 
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                now.time_since_epoch()).count()
                    / 1.0e9;

        traj1->start_time = t_rel + 1e-2;
        tm.add_trajectory(traj1);

        traj2->start_time = traj1->start_time + traj1->duration;
        tm.add_trajectory(traj2);

        traj3->start_time = traj2->start_time + traj2->duration;
        tm.add_trajectory(traj3);

        traj4->start_time = traj3->start_time + traj3->duration;
        tm.add_trajectory(traj4);
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
    }
    std::cout << "Test 8 ended!" << std::endl;
    std::cout << "============================" << std::endl;

    return 0;
}