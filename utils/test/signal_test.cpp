#include <chrono>
#include <iostream>
// #include <unistd>
#include "utils/Utils.hpp"
#include <cstdlib>
#include <thread>
#include <vector>
using namespace std;
using namespace KinovaRobustControl::Utils::Signal;
using namespace chrono_literals;

int main()
{
    int iret = 0;

    auto f = [] {
        // Infinite loop until signal ctrl-c (KILL) received
        while (true)
        {
            cout << "sleeping" << endl;
            this_thread::sleep_for(1s);
        }
    };
    std::vector<std::thread> threads;
    try
    {
        // Register signal handler to handle kill signal
        setupSignalHandlers();
        for (int i = 0; i < 5; i++)
        {
            threads.push_back(std::thread(f));
        }

        for (int i = 0; i < 5; i++)
        {
            threads[i].join();
        }
        iret = EXIT_SUCCESS;
    }
    catch (SignalException &e)
    {
        std::cerr << "SignalException: " << e.what() << std::endl;
        iret = EXIT_FAILURE;
    }
    catch (...)
    {
        cout << "not catched? why?" << endl;
    }
    return (iret);
}
