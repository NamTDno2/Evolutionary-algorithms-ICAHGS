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
    
    int timeViolationCount = 0;
    int capacityViolationCount = 0;
    int energyViolationCount = 0;
    
    std::cout << "Drone specs (heterogeneous fleet):" << std::endl;
    
    for (size_t droneId = 0; droneId < solution.droneRoutes.size(); droneId++) {
        const auto& droneParam = instance.droneParams[droneId];
        
        double maxFlightTime = droneParam.maxFlightTime;
        double maxCapacity = droneParam.maxCapacity;
        double maxEnergy = droneParam.maxEnergy;
        double cruiseSpeed = droneParam.cruiseSpeed;
        double beta = droneParam.beta;
        double gamma = droneParam.gamma;
        
        // Print drone specs
        std::cout << "Drone " << droneId << ": time=" << maxFlightTime << "s cap=" 
                  << maxCapacity << "kg energy=" << maxEnergy << "kJ speed=" 
                  << cruiseSpeed << "m/s | " << solution.droneRoutes[droneId].size() 
                  << " trips" << std::endl;
        
        // Track max values for this drone
        double maxTripTime = 0;
        double maxTripCapacity = 0;
        double maxTripEnergy = 0;
        
        for (size_t j = 0; j < solution.droneRoutes[droneId].size(); j++) {
            const auto& trip = solution.droneRoutes[droneId][j];
            double completionTime = trip.completionTime;
            
            // Calculate distance
            double totalDistance = 0;
            int prevNode = 0;
            for (int custId : trip.customers) {
                totalDistance += instance.getDistance(prevNode, custId);
                prevNode = custId;
            }
            totalDistance += instance.getDistance(prevNode, 0);
            
            // Calculate total demand
            double totalDemand = 0;
            for (int custId : trip.customers) {
                totalDemand += instance.customers[custId - 1].demand;
            }
            
            // Calculate energy consumption
            const int height = 50;
            double takeoffTime = droneParam.takeoffSpeed != 0 ? 
                height / droneParam.takeoffSpeed : 0;
            double landingTime = droneParam.landingSpeed != 0 ? 
                height / droneParam.landingSpeed : 0;
            
            double totalEnergy = 0;
            double currentLoad = totalDemand;
            prevNode = 0;
            
            for (int custId : trip.customers) {
                double dist = instance.getDistance(prevNode, custId);
                double travelTime = dist / cruiseSpeed;
                double power = beta * currentLoad + gamma;
                totalEnergy += power * (takeoffTime + travelTime + landingTime);
                currentLoad -= instance.customers[custId - 1].demand;
                prevNode = custId;
            }
            
            // Return to depot
            double dist = instance.getDistance(prevNode, 0);
            double power = beta * currentLoad + gamma;
            totalEnergy += power * (takeoffTime + (dist / cruiseSpeed) + landingTime);
            totalEnergy /= 1000.0; // Convert to kJ
            
            // Track maximums
            maxTripTime = std::max(maxTripTime, completionTime);
            maxTripCapacity = std::max(maxTripCapacity, totalDemand);
            maxTripEnergy = std::max(maxTripEnergy, totalEnergy);
            
            // Check violations
            if (completionTime > maxFlightTime) {
                std::cout << "  ❌ Trip " << j << " TIME violation: " << completionTime 
                          << "s > " << maxFlightTime << "s" << std::endl;
                timeViolationCount++;
            }
            if (totalDemand > maxCapacity) {
                std::cout << "  ❌ Trip " << j << " CAPACITY violation: " << totalDemand 
                          << "kg > " << maxCapacity << "kg" << std::endl;
                capacityViolationCount++;
            }
            if (totalEnergy > maxEnergy) {
                std::cout << "  ❌ Trip " << j << " ENERGY violation: " << totalEnergy 
                          << "kJ > " << maxEnergy << "kJ" << std::endl;
                energyViolationCount++;
            }
        }
        
        // Print summary for this drone
        std::cout << "  Max usage: time=" << std::fixed << std::setprecision(2) 
                  << maxTripTime << "s (" << (maxTripTime/maxFlightTime*100) << "%), "
                  << "capacity=" << maxTripCapacity << "kg (" 
                  << (maxTripCapacity/maxCapacity*100) << "%), "
                  << "energy=" << std::setprecision(1) << maxTripEnergy << "kJ (" 
                  << (maxTripEnergy/maxEnergy*100) << "%)" << std::endl;
    }
    
    int totalViolations = timeViolationCount + capacityViolationCount + energyViolationCount;
    if (totalViolations > 0) {
        std::cout << "\n❌ Found " << totalViolations << " violations:" << std::endl;
        if (timeViolationCount > 0) 
            std::cout << "   - Time: " << timeViolationCount << std::endl;
        if (capacityViolationCount > 0) 
            std::cout << "   - Capacity: " << capacityViolationCount << std::endl;
        if (energyViolationCount > 0) 
            std::cout << "   - Energy: " << energyViolationCount << std::endl;
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
                   const string& filename, const string& datasetName,
                   double executionTime, int paretoSize, int uniqueSolutions) {
    // Check if file exists to determine append mode
    bool fileExists = ifstream(filename).good();
    ofstream file(filename, ios::app);
    
    if (!file.is_open()) {
        cerr << "Cannot open output file: " << filename << endl;
        return;
    }
    
    // Write header only if file is new
    if (!fileExists) {
        file << "Dataset,SolutionID,CompletionTime,TotalWaitingTime,ExecutionTime,ParetoSize,UniqueSolutions" << endl;
    }
    
    set<pair<double, double>> exportedObjectives;
    int solutionId = 0;
    for (const auto& solution : paretoFront) {
        pair<double, double> objectives = {solution.systemCompletionTime, solution.totalSampleWaitingTime};
        
        if (exportedObjectives.find(objectives) == exportedObjectives.end()) {
            file << datasetName << ","
                 << solutionId++ << "," 
                 << solution.systemCompletionTime << ","
                 << solution.totalSampleWaitingTime << ","
                 << executionTime << ","
                 << paretoSize << ","
                 << uniqueSolutions << endl;
            exportedObjectives.insert(objectives);
        }
    }
    
    file.close();
    cout << "\nUnique results exported to: " << filename << endl;
}

int main(int argc, char* argv[]) {
    auto startTime = clock();
    
    cout << "=== ICAHGS for MSSVTDE ===" << endl;
    
    string filename = "../data/50.10.1.txt";

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
    
    // CẢI TIẾN: Adaptive parameters dựa trên kích thước VÀ số lượng trucks
    int numCustomers = instance.getNumCustomers();
    int numTrucks = instance.numTrucks;
    int populationSize, numEmpires, maxIterations;
    
    if (numCustomers <= 20) {
        populationSize = 50;
        numEmpires = 5;
        maxIterations = 10;
    } else if (numCustomers <= 50) {
        //  Datasets 50C + 30-40T cần iterations cao hơn
        if (numTrucks >= 30) {
            populationSize = 70;    
            numEmpires = 7;
            maxIterations = 15;     
        } else {
            populationSize = 60;    
            numEmpires = 6;
            maxIterations = 12;     
        }
    } else if (numCustomers <= 100) {
        // Datasets 100C + 20-30T cần iterations rất cao
        if (numTrucks >= 20) {
            populationSize = 120;   
            numEmpires = 12;
            maxIterations = 25;     
        } else {
            populationSize = 100;   
            numEmpires = 10;
            maxIterations = 20;     
        }
    } else {  // 200+
        populationSize = 80;    
        numEmpires = 8;
        maxIterations = 18;     
    }
    
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
    
    //  Verify first solution
    if (!paretoFront.empty()) {
        verifySolutionFeasibility(paretoFront[0], instance);
        verifyDroneRouteConstraints(paretoFront[0], instance);
    }
    
    // Extract dataset name from filename (e.g., "data/200.20.1.txt" -> "200.20.1")
    string datasetName = filename;
    size_t lastSlash = datasetName.find_last_of("/\\");
    if (lastSlash != string::npos) {
        datasetName = datasetName.substr(lastSlash + 1);
    }
    size_t lastDot = datasetName.find_last_of(".");
    if (lastDot != string::npos) {
        datasetName = datasetName.substr(0, lastDot);
    }
    
    // Calculate execution time before export
    auto endTime = clock();
    double totalTime = double(endTime - startTime) / CLOCKS_PER_SEC;
    
    exportResults(paretoFront, "results.csv", datasetName, totalTime, paretoFront.size(), solutionsPrinted);
    
    // Quick feasibility check for first solution
    bool isFeasible = false;
    if (!paretoFront.empty()) {
        const Solution& sol = paretoFront[0];
        set<int> servedCustomers;
        for (const auto& route : sol.truckRoutes) {
            for (int cust : route.customers) {
                servedCustomers.insert(cust);
            }
        }
        for (const auto& droneRoute : sol.droneRoutes) {
            for (const auto& trip : droneRoute) {
                for (int cust : trip.customers) {
                    servedCustomers.insert(cust);
                }
            }
        }
        isFeasible = (servedCustomers.size() == (size_t)instance.getNumCustomers());
    }
    
    cout << "\n========== SUMMARY ==========" << endl;
    cout << "Total execution time: " << fixed << setprecision(3) << totalTime << " seconds" << endl;
    cout << "Pareto front size: " << paretoFront.size() << endl;
    cout << "Unique solutions: " << solutionsPrinted << endl;
    cout << "Feasibility: " << (isFeasible ? "YES" : "NO") << endl;
    
    return 0;
}