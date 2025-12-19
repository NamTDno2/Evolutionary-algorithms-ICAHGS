#include "DataStructures.h"
#include "InputReader.h"
#include "ICAHGS.h"
#include "Chromosome.h"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include <ctime>
#include <set>
#include <utility>
#include <random>

using namespace std;

// ✅ HÀM 1: Check Distance Matrix
void verifyDistanceMatrix(const Instance& instance) {
    std::cout << "\n========== DISTANCE VERIFICATION ==========" << std::endl;
    
    double maxDist = 0;
    double minDist = 1e9;
    
    for (int i = 0; i < instance.getNumCustomers() && i < 20; i++) {
        double dist = instance.getDistance(0, i + 1);
        maxDist = std::max(maxDist, dist);
        minDist = std::min(minDist, dist);
        std::cout << "Distance(0," << (i+1) << "): " << std::fixed << std::setprecision(2) 
                  << dist << " meters" << std::endl;
    }
    
    std::cout << "\nDistance range: [" << minDist << ", " << maxDist << "]" << std::endl;
    
    // Check symmetric
    bool symmetric = true;
    for (int i = 1; i <= 5 && i <= instance.getNumCustomers(); i++) {
        for (int j = i+1; j <= 5 && j <= instance.getNumCustomers(); j++) {
            double d1 = instance.getDistance(i, j);
            double d2 = instance.getDistance(j, i);
            if (std::abs(d1 - d2) > 0.01) {
                symmetric = false;
            }
        }
    }
    
    if (symmetric) {
        std::cout << "✅ Distance matrix is SYMMETRIC" << std::endl;
    }
    
    if (maxDist < 100) {
        std::cout << "⚠️  WARNING: Max distance=" << maxDist << " (TOO SMALL!)" << std::endl;
        std::cout << "    Possible: Coordinates in KILOMETERS, not METERS" << std::endl;
    }
}

// ✅ HÀM 2: Check Solution Feasibility
void verifySolutionFeasibility(const Solution& solution, const Instance& instance) {
    std::cout << "\n========== SOLUTION FEASIBILITY ==========" << std::endl;
    
    std::vector<bool> served(instance.getNumCustomers() + 1, false);
    int totalServed = 0;
    int duplicates = 0;
    int notServed = 0;
    
    // Check truck routes
    for (size_t i = 0; i < solution.truckRoutes.size(); i++) {
        const auto& route = solution.truckRoutes[i];
        for (int custId : route.customers) {
            if (custId < 1 || custId > instance.getNumCustomers()) continue;
            if (served[custId]) {
                duplicates++;
            } else {
                served[custId] = true;
                totalServed++;
            }
        }
    }
    
    // Check drone routes
    for (size_t i = 0; i < solution.droneRoutes.size(); i++) {
        for (size_t j = 0; j < solution.droneRoutes[i].size(); j++) {
            for (int custId : solution.droneRoutes[i][j].customers) {
                if (custId < 1 || custId > instance.getNumCustomers()) continue;
                if (served[custId]) {
                    duplicates++;
                } else {
                    served[custId] = true;
                    totalServed++;
                }
            }
        }
    }
    
    // Count not served
    for (int i = 1; i <= instance.getNumCustomers(); i++) {
        if (!served[i]) notServed++;
    }
    
    std::cout << "Total customers: " << instance.getNumCustomers() << std::endl;
    std::cout << "Customers served: " << totalServed << std::endl;
    std::cout << "Customers NOT served: " << notServed << std::endl;
    std::cout << "Duplicate services: " << duplicates << std::endl;
    
    if (totalServed == instance.getNumCustomers() && notServed == 0 && duplicates == 0) {
        std::cout << "✅ FEASIBLE (all customers served exactly once)" << std::endl;
    } else {
        std::cout << "❌ INFEASIBLE" << std::endl;
    }
}

// ✅ HÀM 3: Check Drone Route Constraints
void verifyDroneRouteConstraints(const Solution& solution, const Instance& instance) {
    std::cout << "\n========== DRONE ROUTE CONSTRAINTS ==========" << std::endl;
    
    int violationCount = 0;
    double maxFlightTime = instance.droneParams.maxFlightTime;
    double cruiseSpeed = instance.droneParams.cruiseSpeed;
    
    std::cout << "Max flight time: " << maxFlightTime << "s" << std::endl;
    std::cout << "Cruise speed: " << cruiseSpeed << "m/s" << std::endl << std::endl;
    
    for (size_t i = 0; i < solution.droneRoutes.size(); i++) {
        std::cout << "Drone " << i << ":" << std::endl;
        
        for (size_t j = 0; j < solution.droneRoutes[i].size(); j++) {
            const auto& trip = solution.droneRoutes[i][j];
            double completionTime = trip.completionTime;
            
            // Calculate distance
            double totalDistance = 0;
            int prevNode = 0;
            for (int custId : trip.customers) {
                totalDistance += instance.getDistance(prevNode, custId);
                prevNode = custId;
            }
            totalDistance += instance.getDistance(prevNode, 0);
            
            double expectedTime = totalDistance / cruiseSpeed;
            
            std::cout << "  Trip " << j << ": dist=" << std::fixed << std::setprecision(0) 
                      << totalDistance << "m time=" << std::setprecision(2) 
                      << completionTime << "s";
            
            if (completionTime > maxFlightTime) {
                std::cout << " ❌ VIOLATION";
                violationCount++;
            }
            std::cout << std::endl;
        }
    }
    
    if (violationCount > 0) {
        std::cout << "\n❌ Found " << violationCount << " violations" << std::endl;
    } else {
        std::cout << "\n✅ All constraints satisfied" << std::endl;
    }
}

void printSolution(const Solution& solution, int index) {
    cout << "\n--- Solution " << index << " ---" << endl;
    cout << "System Completion Time: " << fixed << setprecision(2) 
         << solution.systemCompletionTime << " seconds" << endl;
    cout << "Total Sample Waiting Time: " << solution.totalSampleWaitingTime 
         << " seconds" << endl;

    solution.chrom.printGenotype();
    
    cout << "\nTruck Routes:" << endl;
    for (size_t i = 0; i < solution.truckRoutes.size(); i++) {
        const auto& route = solution.truckRoutes[i];
        if (!route.isEmpty()) {
            cout << "  Truck " << i << ": Depot -> ";
            for (int cust : route.customers) {
                cout << cust << " -> ";
            }
            cout << "Depot (Completion: " << route.completionTime 
                 << "s)" << endl;
        }
    }
    
    cout << "\nDrone Routes:" << endl;
    for (size_t i = 0; i < solution.droneRoutes.size(); i++) {
        const auto& trips = solution.droneRoutes[i];
        if (!trips.empty()) {
            cout << "  Drone " << i << ":" << endl;
            for (size_t j = 0; j < trips.size(); j++) {
                cout << "    Trip " << j << ": Depot -> ";
                for (int cust : trips[j].customers) {
                    cout << cust << " -> ";
                }
                cout << "Depot (Completion: " << trips[j].completionTime 
                      << "s)" << endl;
            }
        }
    }
}

void exportResults(const vector<Solution>& paretoFront, 
                   const string& filename) {
    ofstream file(filename);
    
    if (!file.is_open()) {
        cerr << "Cannot open output file: " << filename << endl;
        return;
    }
    
    file << "SolutionID,CompletionTime,TotalWaitingTime" << endl;
    
    set<pair<double, double>> exportedObjectives;
    int solutionId = 0;
    for (const auto& solution : paretoFront) {
        pair<double, double> objectives = {solution.systemCompletionTime, solution.totalSampleWaitingTime};
        
        if (exportedObjectives.find(objectives) == exportedObjectives.end()) {
            file << solutionId++ << "," 
                 << solution.systemCompletionTime << ","
                 << solution.totalSampleWaitingTime << endl;
            exportedObjectives.insert(objectives);
        }
    }
    
    file.close();
    cout << "\nUnique results exported to: " << filename << endl;
}

int main(int argc, char* argv[]) {
    auto startTime = clock();
    
    cout << "=== ICAHGS for MSSVTDE ===" << endl;
    
    string filename = "../data/50.40.1.txt";

    if (argc > 1) {
        filename = argv[1];
    }
    
    Instance instance;
    if (!InputReader::readInstance(filename, instance)) {
        cerr << "Failed to read instance file." << endl;
        return 1;
    }
    
    cout << "\nInstance loaded successfully!" << endl;
    cout << "  Customers: " << instance.getNumCustomers() << endl;
    cout << "  Trucks: " << instance.numTrucks << endl;
    cout << "  Drones: " << instance.numDrones << endl;
    
    //  THÊM: Verify distance matrix
    verifyDistanceMatrix(instance);
    
    int populationSize = 50;
    int numEmpires = 5;
    int maxIterations = 10;
    
    if (argc > 2) populationSize = stoi(argv[2]);
    if (argc > 3) numEmpires = stoi(argv[3]);
    if (argc > 4) maxIterations = stoi(argv[4]);
    
    ICAHGS algorithm(instance, populationSize, numEmpires);
    
    vector<Solution> paretoFront = algorithm.run(maxIterations);
    
    sort(paretoFront.begin(), paretoFront.end(), 
         [](const Solution& a, const Solution& b) {
        if (a.systemCompletionTime != b.systemCompletionTime) {
            return a.systemCompletionTime < b.systemCompletionTime;
        }
        return a.totalSampleWaitingTime < b.totalSampleWaitingTime;
    });
    
    cout << "\n--- Top Unique Solutions ---" << endl;
    set<pair<double, double>> printedObjectives;
    int solutionsPrinted = 0;
    for (const auto& solution : paretoFront) {
        if (solutionsPrinted >= 5) {
            break;
        }
        
        pair<double, double> objectives = {solution.systemCompletionTime, solution.totalSampleWaitingTime};
        
        if (printedObjectives.find(objectives) == printedObjectives.end()) {
            printSolution(solution, solutionsPrinted + 1);
            printedObjectives.insert(objectives);
            solutionsPrinted++;
        }
    }
    
    // ✅ THÊM: Verify first solution
    if (!paretoFront.empty()) {
        verifySolutionFeasibility(paretoFront[0], instance);
        verifyDroneRouteConstraints(paretoFront[0], instance);
    }
    
    exportResults(paretoFront, "results.csv");
    
    auto endTime = clock();
    double totalTime = double(endTime - startTime) / CLOCKS_PER_SEC;
    
    cout << "\n========== SUMMARY ==========" << endl;
    cout << "Total execution time: " << fixed << setprecision(3) << totalTime << " seconds" << endl;
    cout << "Pareto front size: " << paretoFront.size() << endl;
    cout << "Unique solutions: " << solutionsPrinted << endl;
    
    return 0;
}