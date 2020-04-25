//
// Created by mrd on 11/21/19.
//

#ifndef MPC_LINEARIZED_OBSTACLE_H
#define MPC_LINEARIZED_OBSTACLE_H

#include <vector>
#include <string>
#include "../Eigen/Dense"

using Eigen::MatrixXd;

class Obstacle {
private:
    std::string name;
    float radius;
    float dt;
    float maxVelocity;
    Eigen::Vector2d position;
    Eigen::Vector2d velocity;
    std::vector<Eigen::Vector2d> path;

public:
    Obstacle(std::string name, Eigen::Vector2d position, Eigen::Vector2d velocity, float radius=1, float dt=0.1) {
        this->name = name;
        this->position = position;
        this->velocity = velocity;
        this->radius = radius;
        this->dt = dt;
    }

    std::string getName() {
        return name;
    }

    Eigen::Vector2d getPosition() {
        return position;
    }

    Eigen::Vector2d getFuturePosition() {
        return position + velocity*dt;
    }

    Eigen::Vector2d getVelocity() {
        return velocity;
    }

    Eigen::Vector2d getFutureVelocity() {
        return velocity;
    }

    float getRadius() {
        return radius;
    }

    void move() {
        path.push_back(position);
        position = position + velocity*dt;
    }
};


#endif //MPC_LINEARIZED_OBSTACLE_H
