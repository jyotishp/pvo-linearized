#include <iostream>
#include "Eigen/Dense"
#include "Agents/Robot.h"
#include "Agents/Obstacle.h"
#include "Planner/Optimizer.h"

void printStates(Robot robot, Eigen::Vector2d control) {
    std::cout << robot.getName() << ",";
    std::cout << robot.getPosition()[0] << ",";
    std::cout << robot.getPosition()[1] << ",";
    std::cout << robot.getVelocity()[0] << ",";
    std::cout << robot.getVelocity()[1] << ",";
    std::cout << control[0] << ",";
    std::cout << control[1] << std::endl;
}

void printStates(Obstacle obstacle) {
    std::cout << obstacle.getName() << ",";
    std::cout << obstacle.getPosition()[0] << ",";
    std::cout << obstacle.getPosition()[1] << ",";
    std::cout << obstacle.getVelocity()[0] << ",";
    std::cout << obstacle.getVelocity()[1] << ",," << std::endl;
}

int main() {
//    std::cout << ":1" << std::endl;
    Eigen::Vector2d botInitPosition(10, -10);
    Eigen::Vector2d botGoal(-12, 12);
    Robot robot("robot", botInitPosition, botGoal);

//    std::cout << ":2" << std::endl;
    Eigen::Vector2d obstacleInitPosition(-13, 0);
    Eigen::Vector2d obstacleVelocity(1.3, 0);
    Obstacle obstacle("obstacle", obstacleInitPosition, obstacleVelocity);

//    std::cout << ":3" << std::endl;
//    nt noiseSamplesCount, int controlSamplesCount, float xBotDelta, float yBotDelta, float xObsDelta, float yObsDelta, float vxBotDelta, float vyBotDelta, float vxObsDelta, float vyObsDelta, float axDelta, float ayDelta
    Optimizer optimizer(50, 10, 0.224, 0.224, 0.2312, 0.2312, 0.0682, 0.0682, 0.07, 0.07, 0.1312, 0.1312);

    while (!optimizer.goalReached(robot)) {
        Eigen::Vector2d control;
        if (robot.inSensorRange(obstacle)) {
            control = optimizer.getOptimalControl(robot, obstacle);
        } else {
            control = optimizer.getOptimalControl(robot);
        }
        robot.setAcceleration(control);
        obstacle.move();
        printStates(robot, control);
        printStates(obstacle);
    }
    return 0;
}
