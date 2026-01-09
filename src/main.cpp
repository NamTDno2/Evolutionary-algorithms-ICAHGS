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

// Convert solution to route string format (like benchmark)
string solutionToRouteString(const Solution& solution) {
    string routeStr = "";
    
    // Add truck routes
    for (size_t i = 0; i < solution.truckRoutes.size(); i++) {
        for (int cust : solution.truckRoutes[i].customers) {
            routeStr += to_string(cust) + " ";
        }
        routeStr += "|";
    }
    
    // Add separator for drones
    routeStr += "|";
    
    // Add drone routes
    for (size_t i = 0; i < solution.droneRoutes.size(); i++) {
        for (const auto& trip : solution.droneRoutes[i]) {
            for (int cust : trip.customers) {
                routeStr += to_string(cust) + " ";
            }
            routeStr += "|";
        }
        // Add separator between drones if not last drone
        if (i < solution.droneRoutes.size() - 1) {
            routeStr += "|";
        }
    }
    
    return routeStr;
}

void exportResults(const vector<Solution>& paretoFront, 
                   const string& filename, const string& datasetName,
                   double executionTime, int paretoSize, int uniqueSolutions) {
    // Create result_50c_100c_50pct directory for 50% baseline test
    #ifdef _WIN32
        system("if not exist result_50c_100c_50pct mkdir result_50c_100c_50pct");
    #else
        system("mkdir -p result_50c_100c_50pct");
    #endif
    
    // Create output file path: result_50c_100c_50pct/datasetName.txt
    string outputPath = "result_50c_100c_50pct/" + datasetName + ".txt";
    ofstream file(outputPath);
    
    if (!file.is_open()) {
        cerr << "Cannot open output file: " << outputPath << endl;
        return;
    }
    
    // Write benchmark-style header
    file << "Time:0" << endl;
    file << "Last Iter:" << uniqueSolutions << endl;
    file << "Last Update:" << uniqueSolutions << endl;
    file << "Tabu:0" << endl;
    file << "Last Time0" << endl;
    file << uniqueSolutions << endl;
    
    // Export each unique solution
    set<pair<double, double>> exportedObjectives;
    for (const auto& solution : paretoFront) {
        pair<double, double> objectives = {solution.systemCompletionTime, solution.totalSampleWaitingTime};
        
        if (exportedObjectives.find(objectives) == exportedObjectives.end()) {
            string routeStr = solutionToRouteString(solution);
            file << routeStr << endl;
            file << fixed << setprecision(2) << solution.systemCompletionTime << " " 
                 << solution.totalSampleWaitingTime << endl;
            exportedObjectives.insert(objectives);
        }
    }
    
    file.close();
    cout << "\nResults exported to: " << outputPath << endl;
}

int main(int argc, char* argv[]) {
    auto startTime = clock();
    
    cout << "=== ICAHGS for MSSVTDE ===" << endl;
    
    string filename = "../data/20.10.1.txt";

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
    

    verifyDistanceMatrix(instance);
    
    
    int numCustomers = instance.getNumCustomers();
    int numTrucks = instance.numTrucks;
    int populationSize, numEmpires;
    
    // FINAL HYBRID CONFIGURATION (OPTIMAL):
    // - 20C/200C: Use maxEvaluation (proven to work well)
    // - 50C/100C: Use maxIteration (proven superior to maxEvaluation)
    populationSize = 200;
    numEmpires = 3;
    
    // Parse command line arguments BEFORE creating algorithm
    if (argc > 2) populationSize = stoi(argv[2]);
    if (argc > 3) numEmpires = stoi(argv[3]);
    
    ICAHGS algorithm(instance, populationSize, numEmpires);
    SolutionEvaluator::resetCounter();
    
    vector<Solution> paretoFront;
    
    if (numCustomers <= 20) {
        int maxEvaluations = 65000;  
        if (argc > 4) maxEvaluations = stoi(argv[4]);
        paretoFront = algorithm.runWithEvaluationLimit(maxEvaluations);
    } else if (numCustomers <= 50) {
        int maxEvaluations = 1095000;  // 50% baseline (2,190,000 * 0.50)
        if (argc > 4) maxEvaluations = stoi(argv[4]);
        paretoFront = algorithm.runWithEvaluationLimit(maxEvaluations);
    } else if (numCustomers <= 100) {
        int maxEvaluations = 20610000;  // 50% baseline (41,220,000 * 0.50)
        if (argc > 4) maxEvaluations = stoi(argv[4]);
        paretoFront = algorithm.runWithEvaluationLimit(maxEvaluations);
    } else {
        int maxEvaluations = 18800000;  
        if (argc > 4) maxEvaluations = stoi(argv[4]);
        paretoFront = algorithm.runWithEvaluationLimit(maxEvaluations);
    }
    
    int totalEvaluations = SolutionEvaluator::getEvaluationCount();
    
    // Find ideal point (best CT, best WT) from Pareto front
    double idealCT = INF;
    double idealWT = INF;
    for (const auto& sol : paretoFront) {
        idealCT = min(idealCT, sol.systemCompletionTime);
        idealWT = min(idealWT, sol.totalSampleWaitingTime);
    }
    
    // Find nadir point (worst CT, worst WT) for normalization
    double nadirCT = 0;
    double nadirWT = 0;
    for (const auto& sol : paretoFront) {
        nadirCT = max(nadirCT, sol.systemCompletionTime);
        nadirWT = max(nadirWT, sol.totalSampleWaitingTime);
    }
    
    cout << "\n=== IDEAL & NADIR POINTS ===" << endl;
    cout << "Ideal Point: CT=" << fixed << setprecision(2) << idealCT 
         << ", WT=" << idealWT << endl;
    cout << "Nadir Point: CT=" << nadirCT << ", WT=" << nadirWT << endl;
    
    // Select best compromise solution using normalized distance to ideal point
    // This balances both objectives without linear weighting
    double bestDistance = INF;
    Solution bestCompromise;
    int bestIndex = -1;
    
    for (size_t i = 0; i < paretoFront.size(); i++) {
        const auto& sol = paretoFront[i];
        
        // Normalize objectives to [0,1]
        double normCT = (nadirCT > idealCT) ? 
            (sol.systemCompletionTime - idealCT) / (nadirCT - idealCT) : 0;
        double normWT = (nadirWT > idealWT) ? 
            (sol.totalSampleWaitingTime - idealWT) / (nadirWT - idealWT) : 0;
        
        // Euclidean distance to ideal point (0,0) in normalized space
        double distance = sqrt(normCT * normCT + normWT * normWT);
        
        if (distance < bestDistance) {
            bestDistance = distance;
            bestCompromise = sol;
            bestIndex = i;
        }
    }
    
    // Sort Pareto front by distance to ideal (best compromise first)
    sort(paretoFront.begin(), paretoFront.end(), 
         [idealCT, idealWT, nadirCT, nadirWT](const Solution& a, const Solution& b) {
        double normCT_a = (nadirCT > idealCT) ? 
            (a.systemCompletionTime - idealCT) / (nadirCT - idealCT) : 0;
        double normWT_a = (nadirWT > idealWT) ? 
            (a.totalSampleWaitingTime - idealWT) / (nadirWT - idealWT) : 0;
        double dist_a = sqrt(normCT_a * normCT_a + normWT_a * normWT_a);
        
        double normCT_b = (nadirCT > idealCT) ? 
            (b.systemCompletionTime - idealCT) / (nadirCT - idealCT) : 0;
        double normWT_b = (nadirWT > idealWT) ? 
            (b.totalSampleWaitingTime - idealWT) / (nadirWT - idealWT) : 0;
        double dist_b = sqrt(normCT_b * normCT_b + normWT_b * normWT_b);
        
        return dist_a < dist_b;
    });
    
    cout << "\n=== OPTIMIZATION SUMMARY ===" << endl;
    cout << "Total evaluations: " << totalEvaluations << endl;
    cout << "Archive size: " << paretoFront.size() << endl;
    
    if (!paretoFront.empty()) {
        cout << "\n=== BEST COMPROMISE SOLUTION ===" << endl;
        cout << "Distance to ideal: " << fixed << setprecision(4) << bestDistance << endl;
        printSolution(paretoFront[0], 0);  // Best compromise
    }
    
    cout << "\n--- Other Pareto Solutions (up to 4 more) ---" << endl;
    set<pair<double, double>> printedObjectives;
    int solutionsPrinted = 0;
    
    // Add best compromise to printed set
    if (!paretoFront.empty()) {
        printedObjectives.insert({paretoFront[0].systemCompletionTime, 
                                 paretoFront[0].totalSampleWaitingTime});
    }
    
    // Print remaining unique solutions
    for (size_t i = 1; i < paretoFront.size(); i++) {
        const auto& solution = paretoFront[i];
        if (solutionsPrinted >= 4) {
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
    
    // Count unique solutions in Pareto front for export
    set<pair<double, double>> uniqueObjectives;
    for (const auto& sol : paretoFront) {
        uniqueObjectives.insert({sol.systemCompletionTime, sol.totalSampleWaitingTime});
    }
    
    exportResults(paretoFront, "results.csv", datasetName, totalTime, paretoFront.size(), uniqueObjectives.size());
    
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
    cout << "Total evaluations: " << totalEvaluations << endl;
    cout << "Pareto front size: " << paretoFront.size() << endl;
    cout << "Unique solutions: " << solutionsPrinted << endl;
    cout << "Feasibility: " << (isFeasible ? "YES" : "NO") << endl;
    
    return 0;
}