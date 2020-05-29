#include <iostream>
#include <string>
#include <time.h>
#include <vector>
#include <map>
#include <utility>
#include <random>
#include <boost/shared_ptr.hpp>
#include <boost/filesystem.hpp>
#include "matplotlibcpp.h"
#include "world.h"
#include "test.h"

namespace plt = matplotlibcpp;

int main()
{
    clock_t start, end;
    start = clock();

    // testFcl();
    // testDoubleintegratorCost();
    // testDoubleintegratorConnect();
    // testFMT();
    // testDoubleBvpConnect();
    // testDoubleBvpCost();
    // testFMTBvp();
    testProposotionWorld();
    // testFMTLTL();

    end = clock();
    std::cout << "Run time: " << (double)(end - start) / CLOCKS_PER_SEC << "S" << std::endl;
    return 0;
}
