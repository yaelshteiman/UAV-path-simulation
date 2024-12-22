#include <iostream>
#include "Simulation.h"

int main() {
    std::string paramsFile = "../SimParams.ini";
    std::string commandsFile = "../SimCmds.txt";

    try {
        Simulation simulation(paramsFile, commandsFile);
        simulation.run();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}