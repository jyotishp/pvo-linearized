cmake_minimum_required(VERSION 3.15)
project(mpc_linearized)

#set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(CMAKE_CXX_STANDARD 17)

add_executable(mpc_linearized main.cpp Agents/Robot.cpp Agents/Robot.h Agents/Obstacle.h Planner/Optimizer.h)