//
// Created by Yael Shtaiman on 16/12/2024.
//

#include "Simulation.h"
#include <fstream>
#include <sstream>
#include <iostream>

Simulation::Simulation(const std::string& paramsFile, const std::string& cmdsFile)
    : commandQueue([](const Command& a, const Command& b) { return a.time > b.time; }) {
    loadParams(paramsFile);
    loadCommands(cmdsFile);
}

void Simulation::loadParams(const std::string& fileName) {
    std::ifstream file(fileName);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open parameters file: " + fileName);
    }

    int numUAVs = 0;
    float x0 = 0.0f, y0 = 0.0f, z0 = 0.0f, v0 = 0.0f, azimuth = 0.0f, radius = 0.0f;

    std::string line;
    while (std::getline(file, line)) { // Read each line safely
        std::istringstream iss(line);
        std::string key;
        char equals;
        float value;

        if (iss >> key >> equals >> value) { // Parse "key = value" format
            if (key == "Dt") {
                dt = value;
            }
            else if (key == "N_uav") {
                numUAVs = static_cast<int>(value);
            }
            else if (key == "R") {
                radius = value;
            }
            else if (key == "X0") {
                x0 = value;
            }
            else if (key == "Y0") {
                y0 = value;
            }
            else if (key == "Z0") {
                z0 = value;
            }
            else if (key == "V0") {
                v0 = value;
            }
            else if (key == "Az") {
                azimuth = value;
            }
            else if (key == "TimeLim") {
                timeLimit = value;
            }
        }
    }

    // initialize UAVs
    for (int i = 0; i < numUAVs; ++i) {
        uavs.emplace_back(i, x0, y0, z0, v0, azimuth, radius);
    }
    file.close();
}

void Simulation::loadCommands(const std::string& fileName) {
    std::ifstream file(fileName);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open commands file: " + fileName);
    }

    while (file) {
        Command cmd;
        file >> cmd.time >> cmd.uavId >> cmd.targetX >> cmd.targetY;
        if (file) {
            commandQueue.push(cmd);
        }
    }

    file.close();
}

void Simulation::run() {
    float currentTime = 0.0f;

    while (currentTime < timeLimit) {
        std::priority_queue<Command, std::vector<Command>, std::function<bool(Command, Command)>> tempQueue = commandQueue;
        while (!tempQueue.empty()) {
            Command cmd = tempQueue.top();
            tempQueue.pop();
            if (cmd.time <= currentTime) {
                for (auto& uav : uavs) {
                    if (uav.getId() == cmd.uavId) {
                        uav.setTarget(cmd.targetX, cmd.targetY);
                        uav.setHoldingPattern(false);
                    }
                }
            }
        }

        for (auto& uav : uavs) {
            if (!uav.hasTarget()) {
                uav.moveStraight(dt);
            }
            else if (uav.isHoldingPattern()) {
                uav.holdAtPoint(uav.getHoldingCenterX(), uav.getHoldingCenterY(), dt);
            }
            else {
                uav.moveToTarget(uav.getTargetX(), uav.getTargetY(), dt);
            }

            uav.logData("../UAV" + std::to_string(uav.getId()) + ".txt", currentTime);
        }
        currentTime += dt;
    }
}


