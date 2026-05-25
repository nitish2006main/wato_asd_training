#include "costmap_core.hpp"
#include <cmath>

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {}
void CostmapCore::initializeCostmap(){
    for (int i = 0; i < HEIGHT; i++){
        for (int j = 0; j < WIDTH; j++)
        {
            costmap_[i][j]=0;
        }
    }
}
void CostmapCore::markObstacle(int x, int y){
    if (x >= 0 && x < WIDTH && y >= 0 && y < HEIGHT){
        costmap_[y][x] = 100;
    }
}
void CostmapCore::inflateObstacles(){
    double euc_distance = 0.0, cost = 0.0;
    for (int i = 0; i < HEIGHT; i++){
        for (int j = 0; j < WIDTH; j++)
        {
            if (costmap_[i][j] == 100)
            {
                for (int di = -INFLATION_RADIUS; di < INFLATION_RADIUS; di++){
                    for (int dj = -INFLATION_RADIUS; dj < INFLATION_RADIUS; dj++)
                    {
                        euc_distance = sqrt(pow(di, 2) + pow(dj, 2));
                        cost = MAX_COST*(1-(euc_distance/INFLATION_RADIUS));
                        if (i+di >= 0 && i+di < HEIGHT && j+dj >= 0 && j+dj < WIDTH){
                            if (euc_distance < INFLATION_RADIUS)
                            {
                                if (cost > costmap_[i+di][j+dj])
                                {
                                    costmap_[i+di][j+dj] = cost;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
int CostmapCore::getCost(int x, int y){
    return costmap_[y][x];
}
}