//
// Created by mrd on 11/21/19.
//

#ifndef MPC_LINEARIZED_ROBOT_H
#define MPC_LINEARIZED_ROBOT_H

#include <iostream>
#include <vector>
#include <string>
#include "../Eigen/Dense"
#include "Obstacle.h"

class Robot {
private:
    std::string name;
    float radius;
    float dt;
    Eigen::Vector2d position;
    Eigen::Vector2d velocity;
    Eigen::Vector2d acceleration;
    Eigen::Vector2d maxVelocity;
    Eigen::Vector2d goal;
    std::vector<Eigen::Vector2d> path;
    std::vector<Eigen::Vector2d> velocities;
    std::vector<Eigen::Vector2d> accelerations;

public:
    Robot(std::string name, Eigen::Vector2d position, Eigen::Vector2d goal, float radius=1, float dt=0.1) {
        this->name = name;
        this->position = position;
        this->goal = goal;
        this->radius = radius;
        this->dt = dt;
    }

    std::string getName() {
        return name;
    }

    Eigen::Vector2d getPosition() {
        return position;
    }

    Eigen::Vector2d getFuturePosition(Eigen::Vector2d control) {
        return position + velocity*dt + 0.5*control*dt*dt;
    }

    Eigen::Vector2d getFuturePosition() {
        return position + velocity*dt + 0.5*acceleration*dt*dt;
    }

    Eigen::Vector2d getVelocity() {
        return velocity;
    }

    Eigen::Vector2d getFutureVelocity(Eigen::Vector2d control) {
        return velocity + control*dt;
    }

    float getRadius() {
        return radius;
    }

    float getTimeStep() {
        return dt;
    }

    Eigen::Vector2d getGoal() {
        return goal;
    }

    void setPosition(Eigen::Vector2d currentPosition) {
        path.push_back(position);
        position = currentPosition;
    }

    void setVelocity(Eigen::Vector2d currentVelocity) {
        velocities.push_back(velocity);
        velocity = currentVelocity;
        Eigen::Vector2d currentPosition = position + velocity*dt;
        setPosition(currentPosition);
    }

    void setAcceleration(Eigen::Vector2d control) {
        accelerations.push_back(acceleration);
        acceleration = control;
        Eigen::Vector2d currentVelocity = velocity + 0.5*acceleration*dt;
        double maxVel = 1.8;
        currentVelocity[0] = std::max(-maxVel, currentVelocity[0]);
        currentVelocity[1] = std::max(-maxVel, currentVelocity[1]);
        currentVelocity[0] = std::min(maxVel, currentVelocity[0]);
        currentVelocity[1] = std::min(maxVel, currentVelocity[1]);
        setVelocity(currentVelocity);
    }

    bool inSensorRange(Obstacle obstacle, float sensorRange=36) {
        float distance = (position - obstacle.getPosition()).array().pow(2).sum();
        float fieldOfView = (position - obstacle.getPosition()).transpose() * (velocity - obstacle.getVelocity());
        if (distance < sensorRange && fieldOfView < 0)
            return true;
        return false;
    }

    std::vector<Eigen::Vector2d> sampleControls(int count) {
        std::vector<Eigen::Vector2d> sampledControls;
        float bound = 3, mid = count/2;
        for (int i = 0; i < count; i++) {
            for (int j = 0; j < count; j++) {
                Eigen::Vector2d control(((i-mid)/mid)*bound, ((j-mid)/mid)*bound);
                sampledControls.push_back(control);
            }
        }
        return sampledControls;
    }
};


#endif //MPC_LINEARIZED_ROBOT_H
