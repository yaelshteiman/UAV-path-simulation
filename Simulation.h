//
// Created by Yael Shtaiman on 16/12/2024.
//

#ifndef SIMULATION_H
#define SIMULATION_H

#include <vector>
#include <queue>
#include <functional>
#include "UAV.h"

struct Command {
    float time;
    int uavId;
    float targetX;
    float targetY;
};

class Simulation {
private:
    std::vector<UAV> uavs;
    float dt;
    float timeLimit;
    std::priority_queue<Command, std::vector<Command>, std::function<bool(Command, Command)>> commandQueue;

    void loadParams(const std::string& fileName); // parse simulation parameters
    void loadCommands(const std::string& fileName); // parse commands

public:
    Simulation(const std::string& paramsFile, const std::string& cmdsFile);
    void run();
};



#endif //SIMULATION_H

