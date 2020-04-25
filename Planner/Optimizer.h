//
// Created by mrd on 11/22/19.
//

#ifndef MPC_LINEARIZED_OPTIMIZER_H
#define MPC_LINEARIZED_OPTIMIZER_H

#include <map>
#include <vector>
#include <string>
#include "../Agents/Robot.h"

class Optimizer {
private:
//    std::map<std::string, std::vector<float>> meanCoefficients, stdCoefficients;
    float noiseSamplesCount, controlSamplesCount;
    float xBotDelta, yBotDelta, xObsDelta, yObsDelta, vxBotDelta, vyBotDelta, vxObsDelta, vyObsDelta, axDelta, ayDelta;

public:
    Optimizer(int noiseSamplesCount, int controlSamplesCount, float xBotDelta, float yBotDelta, float xObsDelta,
            float yObsDelta, float vxBotDelta, float vyBotDelta, float vxObsDelta, float vyObsDelta, float axDelta,
            float ayDelta) {
        this->noiseSamplesCount = noiseSamplesCount;
        this->controlSamplesCount = controlSamplesCount;
        this->xBotDelta = xBotDelta;
        this->yBotDelta = yBotDelta;
        this->xObsDelta = xObsDelta;
        this->yObsDelta = yObsDelta;
        this->vxBotDelta = vxBotDelta;
        this->vyBotDelta = vyBotDelta;
        this->vxObsDelta = vxObsDelta;
        this->vyObsDelta = vyObsDelta;
        this->axDelta = axDelta;
        this->ayDelta = ayDelta;
    }

    double Power(float a, float b) {
        return std::pow(a, b);
    }

    double Sqrt(double a) {
        return std::sqrt(a);
    }

    double meanCollisionCone(Robot robot, Obstacle obstacle, Eigen::Vector2d control) {
        Eigen::Vector2d robotPosition = robot.getFuturePosition(control);
        float mean_xr = robotPosition[0];
        float mean_yr = robotPosition[1];
        Eigen::Vector2d obstaclePosition = obstacle.getFuturePosition();
        float mean_xob = obstaclePosition[0];
        float mean_yob = obstaclePosition[1];
        Eigen::Vector2d robotVelocity = robot.getFutureVelocity(control);
        float mean_xrdot = robotVelocity[0];
        float mean_yrdot = robotVelocity[1];
        Eigen::Vector2d obstacleVelocity = obstacle.getFutureVelocity();
        float mean_xobdot = obstacleVelocity[0];
        float mean_yobdot = obstacleVelocity[1];
        float mean_xrddot = control[0];
        float mean_yrddot = control[1];
        float delt = robot.getTimeStep();
        float R = robot.getRadius() + obstacle.getRadius();
        return -Power((-mean_xobdot + delt*mean_xrddot + mean_xrdot)*(-mean_xob + mean_xr + 0.5*Power(delt,2)*mean_xrddot + delt*mean_xrdot) + (-mean_yobdot + delt*mean_yrddot + mean_yrdot)*(-mean_yob + mean_yr + 0.5*Power(delt,2)*mean_yrddot + delt*mean_yrdot),2) +        (Power(-mean_xobdot + delt*mean_xrddot + mean_xrdot,2) + Power(-mean_yobdot + delt*mean_yrddot + mean_yrdot,2))*        (Power(-mean_xob + mean_xr + 0.5*Power(delt,2)*mean_xrddot + delt*mean_xrdot,2) + Power(-mean_yob + mean_yr + 0.5*Power(delt,2)*mean_yrddot + delt*mean_yrdot,2) - Power(R,2));
    }

    double stdCollisionCone(Robot robot, Obstacle obstacle, Eigen::Vector2d control) {
        Eigen::Vector2d robotPosition = robot.getFuturePosition(control);
        float mean_xr = robotPosition[0];
        float mean_yr = robotPosition[1];
        Eigen::Vector2d obstaclePosition = obstacle.getFuturePosition();
        float mean_xob = obstaclePosition[0];
        float mean_yob = obstaclePosition[1];
        Eigen::Vector2d robotVelocity = robot.getFutureVelocity(control);
        float mean_xrdot = robotVelocity[0];
        float mean_yrdot = robotVelocity[1];
        Eigen::Vector2d obstacleVelocity = obstacle.getFutureVelocity();
        float mean_xobdot = obstacleVelocity[0];
        float mean_yobdot = obstacleVelocity[1];
        float mean_xrddot = control[0];
        float mean_yrddot = control[1];
        float delt = robot.getTimeStep();
        float R = robot.getRadius() + obstacle.getRadius();
        float stddevxr = xBotDelta;
        float stddevyobdot = vyObsDelta;
        float stddevyr = yBotDelta;
        float stddevxrdot = vxBotDelta;
        float stddevyrdot = vyBotDelta;
        float stddevxrddot = axDelta;
        float stddevyrddot = ayDelta;
        float stddevxob = xObsDelta;
        float stddevyob = yObsDelta;
        float stddevxobdot = vxObsDelta;
        return Sqrt(Power(-2*(-mean_xob + mean_xr + delt*(0.5*delt*mean_xrddot + mean_xrdot))*(Power(-mean_xobdot + delt*mean_xrddot + mean_xrdot,2) + Power(-mean_yobdot + delt*mean_yrddot + mean_yrdot,2)) -         2*(mean_xobdot - delt*mean_xrddot - mean_xrdot)*((-mean_xobdot + delt*mean_xrddot + mean_xrdot)*(-mean_xob + mean_xr + delt*(0.5*delt*mean_xrddot + mean_xrdot)) +            (-mean_yobdot + delt*mean_yrddot + mean_yrdot)*(-mean_yob + mean_yr + delt*(0.5*delt*mean_yrddot + mean_yrdot))),2)*Power(stddevxob,2) +      Power(-2*(mean_xob - mean_xr + delt*(-0.5*delt*mean_xrddot - mean_xrdot))*((-mean_xobdot + delt*mean_xrddot + mean_xrdot)*(-mean_xob + mean_xr + delt*(0.5*delt*mean_xrddot + mean_xrdot)) +            (-mean_yobdot + delt*mean_yrddot + mean_yrdot)*(-mean_yob + mean_yr + delt*(0.5*delt*mean_yrddot + mean_yrdot))) -         2*(-mean_xobdot + delt*mean_xrddot + mean_xrdot)*(Power(-mean_xob + mean_xr + delt*(0.5*delt*mean_xrddot + mean_xrdot),2) + Power(-mean_yob + mean_yr + delt*(0.5*delt*mean_yrddot + mean_yrdot),2) - Power(R,2)),2)*Power(stddevxobdot,2) +      Power(2*(-mean_xob + mean_xr + delt*(0.5*delt*mean_xrddot + mean_xrdot))*(Power(-mean_xobdot + delt*mean_xrddot + mean_xrdot,2) + Power(-mean_yobdot + delt*mean_yrddot + mean_yrdot,2)) -         2*(-mean_xobdot + delt*mean_xrddot + mean_xrdot)*((-mean_xobdot + delt*mean_xrddot + mean_xrdot)*(-mean_xob + mean_xr + delt*(0.5*delt*mean_xrddot + mean_xrdot)) +            (-mean_yobdot + delt*mean_yrddot + mean_yrdot)*(-mean_yob + mean_yr + delt*(0.5*delt*mean_yrddot + mean_yrdot))),2)*Power(stddevxr,2) +      Power(delt,2)*Power(1.*delt*(-mean_xob + mean_xr + delt*(0.5*delt*mean_xrddot + mean_xrdot))*(Power(-mean_xobdot + delt*mean_xrddot + mean_xrdot,2) + Power(-mean_yobdot + delt*mean_yrddot + mean_yrdot,2)) -         2*(-mean_xob + mean_xr + delt*(-0.5*mean_xobdot + 1.*delt*mean_xrddot + 1.5*mean_xrdot))*((-mean_xobdot + delt*mean_xrddot + mean_xrdot)*(-mean_xob + mean_xr + delt*(0.5*delt*mean_xrddot + mean_xrdot)) +            (-mean_yobdot + delt*mean_yrddot + mean_yrdot)*(-mean_yob + mean_yr + delt*(0.5*delt*mean_yrddot + mean_yrdot))) +         2*(-mean_xobdot + delt*mean_xrddot + mean_xrdot)*(Power(-mean_xob + mean_xr + delt*(0.5*delt*mean_xrddot + mean_xrdot),2) + Power(-mean_yob + mean_yr + delt*(0.5*delt*mean_yrddot + mean_yrdot),2) - Power(R,2)),2)*Power(stddevxrddot,2) +      Power(2*delt*(-mean_xob + mean_xr + delt*(0.5*delt*mean_xrddot + mean_xrdot))*(Power(-mean_xobdot + delt*mean_xrddot + mean_xrdot,2) + Power(-mean_yobdot + delt*mean_yrddot + mean_yrdot,2)) -         2*(-1.*mean_xob + mean_xr + delt*(-1.*mean_xobdot + 1.5*delt*mean_xrddot + 2.*mean_xrdot))*((-mean_xobdot + delt*mean_xrddot + mean_xrdot)*(-mean_xob + mean_xr + delt*(0.5*delt*mean_xrddot + mean_xrdot)) +            (-mean_yobdot + delt*mean_yrddot + mean_yrdot)*(-mean_yob + mean_yr + delt*(0.5*delt*mean_yrddot + mean_yrdot))) +         2*(-mean_xobdot + delt*mean_xrddot + mean_xrdot)*(Power(-mean_xob + mean_xr + delt*(0.5*delt*mean_xrddot + mean_xrdot),2) + Power(-mean_yob + mean_yr + delt*(0.5*delt*mean_yrddot + mean_yrdot),2) - Power(R,2)),2)*Power(stddevxrdot,2) +      Power(-2*(-mean_yob + mean_yr + delt*(0.5*delt*mean_yrddot + mean_yrdot))*(Power(-mean_xobdot + delt*mean_xrddot + mean_xrdot,2) + Power(-mean_yobdot + delt*mean_yrddot + mean_yrdot,2)) -         2*(mean_yobdot - delt*mean_yrddot - mean_yrdot)*((-mean_xobdot + delt*mean_xrddot + mean_xrdot)*(-mean_xob + mean_xr + delt*(0.5*delt*mean_xrddot + mean_xrdot)) +            (-mean_yobdot + delt*mean_yrddot + mean_yrdot)*(-mean_yob + mean_yr + delt*(0.5*delt*mean_yrddot + mean_yrdot))),2)*Power(stddevyob,2) +      Power(-2*(mean_yob - mean_yr + delt*(-0.5*delt*mean_yrddot - mean_yrdot))*((-mean_xobdot + delt*mean_xrddot + mean_xrdot)*(-mean_xob + mean_xr + delt*(0.5*delt*mean_xrddot + mean_xrdot)) +            (-mean_yobdot + delt*mean_yrddot + mean_yrdot)*(-mean_yob + mean_yr + delt*(0.5*delt*mean_yrddot + mean_yrdot))) -         2*(-mean_yobdot + delt*mean_yrddot + mean_yrdot)*(Power(-mean_xob + mean_xr + delt*(0.5*delt*mean_xrddot + mean_xrdot),2) + Power(-mean_yob + mean_yr + delt*(0.5*delt*mean_yrddot + mean_yrdot),2) - Power(R,2)),2)*Power(stddevyobdot,2) +      Power(2*(-mean_yob + mean_yr + delt*(0.5*delt*mean_yrddot + mean_yrdot))*(Power(-mean_xobdot + delt*mean_xrddot + mean_xrdot,2) + Power(-mean_yobdot + delt*mean_yrddot + mean_yrdot,2)) -         2*(-mean_yobdot + delt*mean_yrddot + mean_yrdot)*((-mean_xobdot + delt*mean_xrddot + mean_xrdot)*(-mean_xob + mean_xr + delt*(0.5*delt*mean_xrddot + mean_xrdot)) +            (-mean_yobdot + delt*mean_yrddot + mean_yrdot)*(-mean_yob + mean_yr + delt*(0.5*delt*mean_yrddot + mean_yrdot))),2)*Power(stddevyr,2) +      Power(delt,2)*Power(1.*delt*(-mean_yob + mean_yr + delt*(0.5*delt*mean_yrddot + mean_yrdot))*(Power(-mean_xobdot + delt*mean_xrddot + mean_xrdot,2) + Power(-mean_yobdot + delt*mean_yrddot + mean_yrdot,2)) -         2*(-mean_yob + mean_yr + delt*(-0.5*mean_yobdot + 1.*delt*mean_yrddot + 1.5*mean_yrdot))*((-mean_xobdot + delt*mean_xrddot + mean_xrdot)*(-mean_xob + mean_xr + delt*(0.5*delt*mean_xrddot + mean_xrdot)) +            (-mean_yobdot + delt*mean_yrddot + mean_yrdot)*(-mean_yob + mean_yr + delt*(0.5*delt*mean_yrddot + mean_yrdot))) +         2*(-mean_yobdot + delt*mean_yrddot + mean_yrdot)*(Power(-mean_xob + mean_xr + delt*(0.5*delt*mean_xrddot + mean_xrdot),2) + Power(-mean_yob + mean_yr + delt*(0.5*delt*mean_yrddot + mean_yrdot),2) - Power(R,2)),2)*Power(stddevyrddot,2) +      Power(2*delt*(-mean_yob + mean_yr + delt*(0.5*delt*mean_yrddot + mean_yrdot))*(Power(-mean_xobdot + delt*mean_xrddot + mean_xrdot,2) + Power(-mean_yobdot + delt*mean_yrddot + mean_yrdot,2)) -         2*(-1.*mean_yob + mean_yr + delt*(-1.*mean_yobdot + 1.5*delt*mean_yrddot + 2.*mean_yrdot))*((-mean_xobdot + delt*mean_xrddot + mean_xrdot)*(-mean_xob + mean_xr + delt*(0.5*delt*mean_xrddot + mean_xrdot)) +            (-mean_yobdot + delt*mean_yrddot + mean_yrdot)*(-mean_yob + mean_yr + delt*(0.5*delt*mean_yrddot + mean_yrdot))) +         2*(-mean_yobdot + delt*mean_yrddot + mean_yrdot)*(Power(-mean_xob + mean_xr + delt*(0.5*delt*mean_xrddot + mean_xrdot),2) + Power(-mean_yob + mean_yr + delt*(0.5*delt*mean_yrddot + mean_yrdot),2) - Power(R,2)),2)*Power(stddevyrdot,2));
    }

    Eigen::Vector2d desiredVelocity(Robot robot, float maxVelocity) {
        Eigen::Vector2d desiredVelocityVector = robot.getGoal() - robot.getPosition();
        return desiredVelocityVector*maxVelocity/Sqrt(desiredVelocityVector.array().pow(2).sum());
    }

    double desiredVelocityCost(Robot robot, Eigen::Vector2d control, float maxVelocity=1.5) {
        return (robot.getVelocity() + control*robot.getTimeStep() - desiredVelocity(robot, maxVelocity)).array().pow(2).sum();
    }

    double getCost(Robot robot, Obstacle obstacle, Eigen::Vector2d control, float k=1.5) {
        return -meanCollisionCone(robot, obstacle, control) + k*stdCollisionCone(robot, obstacle, control) + desiredVelocityCost(robot, control);
    }

    Eigen::Vector2d getOptimalControl(Robot robot, Obstacle obstacle) {
        std::vector<Eigen::Vector2d> sampledControls = robot.sampleControls(10);
        double minCost = 10e9;
        int optimalControl;
        for (int i = 0; i < 100; i++) {
            double cost = getCost(robot, obstacle, sampledControls[i]);
            if (cost < minCost) {
                minCost = cost;
                optimalControl = i;
            }
        }
        return sampledControls[optimalControl];
    }

    Eigen::Vector2d getOptimalControl(Robot robot) {
        std::vector<Eigen::Vector2d> sampledControls = robot.sampleControls(10);
        double minCost = 10e9;
        int optimalControl;
        for (int i = 0; i < 100; i++) {
            double cost = desiredVelocityCost(robot, sampledControls[i]);
            if (cost < minCost) {
                minCost = cost;
                optimalControl = i;
            }
        }
        return sampledControls[optimalControl];
    }

    bool goalReached(Robot robot) {
        float tolerance = 0.1;
        if ((robot.getPosition() - robot.getGoal()).array().pow(2).sum() < tolerance)
            return true;
        return false;
    }

};


#endif //MPC_LINEARIZED_OPTIMIZER_H
