#include "DataStructures.h"
#include "InputReader.h"
#include "ICAHGS.h"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm> // Cần cho hàm min
#include <ctime>     // Cần cho hàm clock

using namespace std;

void printSolution(const Solution& solution, int index) {
    cout << "\n--- Solution " << index << " ---" << endl;
    cout << "System Completion Time: " << fixed << setprecision(2) 
         << solution.systemCompletionTime << " seconds" << endl;
    cout << "Total Sample Waiting Time: " << solution.totalSampleWaitingTime 
         << " seconds" << endl;
    
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
    
    for (size_t i = 0; i < paretoFront.size(); i++) {
        file << i << "," 
             << paretoFront[i].systemCompletionTime << ","
             << paretoFront[i].totalSampleWaitingTime << endl;
    }
    
    file.close();
    cout << "\nResults exported to: " << filename << endl;
}

int main(int argc, char* argv[]) {
    cout << "=== ICAHGS for MSSVTDE ===" << endl;
    
    string filename = "data/6.5.1.txt";
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
    
    int populationSize = 50;
    int numEmpires = 5;
    int maxIterations = 100;
    
    if (argc > 2) populationSize = stoi(argv[2]);
    if (argc > 3) numEmpires = stoi(argv[3]);
    if (argc > 4) maxIterations = stoi(argv[4]);
    
    ICAHGS algorithm(instance, populationSize, numEmpires);
    
    auto startTime = clock();
    vector<Solution> paretoFront = algorithm.run(maxIterations);
    auto endTime = clock();
    
    double elapsedTime = double(endTime - startTime) / CLOCKS_PER_SEC;
    
    cout << "\n=== Results ===" << endl;
    cout << "Computation time: " << elapsedTime << " seconds" << endl;
    cout << "Pareto front size: " << paretoFront.size() << endl;
    
    // Sắp xếp Pareto front để hiển thị kết quả đa dạng
    sort(paretoFront.begin(), paretoFront.end(), 
              [](const Solution& a, const Solution& b) {
        if (a.systemCompletionTime != b.systemCompletionTime) {
            return a.systemCompletionTime < b.systemCompletionTime;
        }
        return a.totalSampleWaitingTime < b.totalSampleWaitingTime;
    });

    // In top 5 solutions
    int numToPrint = min(5, static_cast<int>(paretoFront.size()));
    for (int i = 0; i < numToPrint; i++) {
        printSolution(paretoFront[i], i + 1);
    }
    
    // Export results
    exportResults(paretoFront, "results.csv");
    
    return 0;
}
