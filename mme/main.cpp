#include "PointCloudMapQualityEvaluation.h"
#include "Timer.hpp"
#include <iostream>

int main()
{
    PointCloudMapQualityEvaluation mqe;
    mqe.LoadMap("/home/will/code/slam/catkin_ws/src/fastlio_localization/PCD/static_map_full.pcd");
    Timer timer;
    timer.start();

    std::cout << "mme = " << mqe.CalculateMeanMapEntropyMetrics(0.3) << std::endl;

    timer.elapsedByLast();
    return 0;
}
