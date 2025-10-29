#include "DataStructures.h"
#include "InputReader.h"
#include "ICAHGS.h"
#include <iostream>
#include <iomanip>
#include <fstream>

void printSolution(const Solution& solution, int index) {
    std::cout << "\n--- Solution " << index << " ---" << std::endl;
    std::cout << "System Completion Time: " << std::fixed << std::setprecision(2) 
              << solution.systemCompletionTime << " seconds" << std::endl;
    std::cout << "Total Sample Waiting Time: " << solution.totalSampleWaitingTime 
              << " seconds" << std::endl;
    
    std::cout << "\nTruck Routes:" << std::endl;
    for (size_t i = 0; i < solution.truckRoutes.size(); i++) {
        const auto& route = solution.truckRoutes[i];
        if (!route.isEmpty()) {
            std::cout << "  Truck " << i << ": Depot -> ";
            for (int cust : route.customers) {
                std::cout << cust << " -> ";
            }
            std::cout << "Depot (Completion: " << route.completionTime 
                      << "s)" << std::endl;
        }
    }
    
    std::cout << "\nDrone Routes:" << std::endl;
    for (size_t i = 0; i < solution.droneRoutes.size(); i++) {
        const auto& trips = solution.droneRoutes[i];
        if (!trips.empty()) {
            std::cout << "  Drone " << i << ":" << std::endl;
            for (size_t j = 0; j < trips.size(); j++) {
                std::cout << "    Trip " << j << ": Depot -> ";
                for (int cust : trips[j].customers) {
                    std::cout << cust << " -> ";
                }
                std::cout << "Depot (Completion: " << trips[j].completionTime 
                          << "s)" << std::endl;
            }
        }
    }
}

void exportResults(const std::vector<Solution>& paretoFront, 
                  const std::string& filename) {
    std::ofstream file(filename);
    
    if (!file.is_open()) {
        std::cerr << "Cannot open output file: " << filename << std::endl;
        return;
    }
    
    file << "SolutionID,CompletionTime,TotalWaitingTime" << std::endl;
    
    for (size_t i = 0; i < paretoFront.size(); i++) {
        file << i << "," 
             << paretoFront[i].systemCompletionTime << ","
             << paretoFront[i].totalSampleWaitingTime << std::endl;
    }
    
    file.close();
    std::cout << "\nResults exported to: " << filename << std::endl;
}

int main(int argc, char* argv[]) {
    std::cout << "=== ICAHGS for MSSVTDE ===" << std::endl;
    
    // Read instance
    std::string filename = "data/instance_example.txt";
    if (argc > 1) {
        filename = argv;
    }
    
    Instance instance;
    if (!InputReader::readInstance(filename, instance)) {
        std::cerr << "Failed to read instance file." << std::endl;
        return 1;
    }
    
    std::cout << "\nInstance loaded successfully!" << std::endl;
    std::cout << "  Customers: " << instance.getNumCustomers() << std::endl;
    std::cout << "  Trucks: " << instance.numTrucks << std::endl;
    std::cout << "  Drones: " << instance.numDrones << std::endl;
    
    // Run ICAHGS
    int populationSize = 50;
    int numEmpires = 5;
    int maxIterations = 100;
    
    if (argc > 2) populationSize = std::stoi(argv);
    if (argc > 3) numEmpires = std::stoi(argv);
    if (argc > 4) maxIterations = std::stoi(argv);
    
    ICAHGS algorithm(instance, populationSize, numEmpires);
    
    auto startTime = std::clock();
    std::vector<Solution> paretoFront = algorithm.run(maxIterations);
    auto endTime = std::clock();
    
    double elapsedTime = double(endTime - startTime) / CLOCKS_PER_SEC;
    
    std::cout << "\n=== Results ===" << std::endl;
    std::cout << "Computation time: " << elapsedTime << " seconds" << std::endl;
    std::cout << "Pareto front size: " << paretoFront.size() << std::endl;
    
    // Print top 5 solutions
    int numToPrint = std::min(5, static_cast<int>(paretoFront.size()));
    for (int i = 0; i < numToPrint; i++) {
        printSolution(paretoFront[i], i + 1);
    }
    
    // Export results
    exportResults(paretoFront, "results.csv");
    
    return 0;
}
