#ifndef DATASTRUCTURES_H
#define DATASTRUCTURES_H


#include <vector>
#include <string>
#include <limits>
#include <cmath> 
#include <cstdint>
#include <iostream>
#include "Chromosome.h"
using namespace std;


// Constants
const double INF = std::numeric_limits<double>::infinity();


// Customer structure
struct Customer {
    int id;
    double x, y;              // Coordinates
    double demand;            // Weight of sample
    bool isStaffOnly;         // True if must be served by technician
    double serviceTimeTruck;  // Service time by truck (seconds)
    double serviceTimeDrone;  // Service time by drone (seconds)
    
    Customer() : id(0), x(0), y(0), demand(0), isStaffOnly(false), 
                 serviceTimeTruck(0), serviceTimeDrone(0) {}
};


// Time interval for time-dependent speed
struct TimeInterval {
    double startTime;  // Ta
    double endTime;    // Ta+1
    double sigma;      // hệ số tỏng khoảng thời gian đó
    
    TimeInterval() : startTime(0), endTime(0), sigma(1.0) {}
    TimeInterval(double s, double e, double sig) 
        : startTime(s), endTime(e), sigma(sig) {}
};


// Drone parameters
struct DroneParams {
    double maxCapacity;     // Md (kg)
    double maxEnergy;       // E (kJ)
    double takeoffSpeed;    // m/s cất cánh
    double cruiseSpeed;     // m/s bay
    double landingSpeed;    // m/s hạ cánh
    double beta;            // W/kg hệ số năng lượng theo tải
    double gamma;           // W - tiêu hao năng lượng nền
    double maxFlightTime;   // thời gian bay tối đa
    
    DroneParams() : maxCapacity(0), maxEnergy(0), takeoffSpeed(0), 
                    cruiseSpeed(0), landingSpeed(0), beta(0), gamma(0),
                    maxFlightTime(0) {}
};


// Truck parameters
struct TruckParams {
    double maxSpeed;  // Vmax (m/s)
    vector<TimeInterval> timeIntervals;  // danh sách các khoảng thời gian, tắc đường
    
    TruckParams() : maxSpeed(0) {}
};


// Instance data
struct Instance {
    int numTrucks;
    int numDrones;
    vector<Customer> customers;
    DroneParams droneParams;
    TruckParams truckParams;
    
    // Depot is always at (0, 0)
    double depotX = 0.0;
    double depotY = 0.0;
    
    // ✅ NEW: Distance cache matrix
    std::vector<std::vector<double>> distanceMatrix;
    
    int getNumCustomers() const { return customers.size(); }
    
    // Calculate Euclidean distance (4-arg version)
    double getDistance(double x1, double y1, double x2, double y2) const {
        double dx = x2 - x1;
        double dy = y2 - y1;
        return sqrt(dx * dx + dy * dy);
    }
    
    // ✅ NEW: Use cache for 2-arg version
    double getDistance(int custId1, int custId2) const {
        // Check cache validity
        if (custId1 < 0 || custId1 >= (int)distanceMatrix.size() ||
            custId2 < 0 || custId2 >= (int)distanceMatrix.size()) {
            // Fallback to calculation if cache not ready
            if (custId1 == 0) { // Depot
                return getDistance(depotX, depotY, 
                                 customers[custId2-1].x, customers[custId2-1].y);
            }
            if (custId2 == 0) { // Depot
                return getDistance(customers[custId1-1].x, customers[custId1-1].y,
                                 depotX, depotY);
            }
            return getDistance(customers[custId1-1].x, customers[custId1-1].y,
                             customers[custId2-1].x, customers[custId2-1].y);
        }
        // Return from cache (very fast!)
        return distanceMatrix[custId1][custId2];
    }
    
    // ✅ NEW: Build distance cache
    void buildDistanceCache() {
        int numNodes = getNumCustomers() + 1;  // +1 for depot (node 0)
        
        // Resize matrix
        distanceMatrix.resize(numNodes, std::vector<double>(numNodes, 0.0));
        
        std::cout << "Building distance cache for " << numNodes << " nodes..." 
                  << std::endl;
        
        // Pre-compute all distances
        for (int i = 0; i < numNodes; i++) {
            for (int j = 0; j < numNodes; j++) {
                if (i == j) {
                    distanceMatrix[i][j] = 0.0;
                } else {
                    double dx, dy;
                    
                    if (i == 0) {
                        // From depot
                        dx = depotX - customers[j - 1].x;
                        dy = depotY - customers[j - 1].y;
                    } else if (j == 0) {
                        // To depot
                        dx = customers[i - 1].x - depotX;
                        dy = customers[i - 1].y - depotY;
                    } else {
                        // Between customers
                        dx = customers[i - 1].x - customers[j - 1].x;
                        dy = customers[i - 1].y - customers[j - 1].y;
                    }
                    
                    distanceMatrix[i][j] = std::sqrt(dx * dx + dy * dy);
                }
            }
        }
        
        std::cout << "Distance cache built (" << numNodes << "x" << numNodes 
                  << ") - Size: " 
                  << (numNodes * numNodes * 8 / 1024) << "KB" << std::endl;
    }
};


// Route structure
struct Route {
    vector<int> customers;  // Customer IDs 
    double completionTime;
    double totalWaitingTime;
    
    Route() : completionTime(0), totalWaitingTime(0) {}
    
    void clear() {
        customers.clear();
        completionTime = 0;
        totalWaitingTime = 0;
    }
    
    bool isEmpty() const { return customers.empty(); }
    int size() const { return customers.size(); }
};


// Solution structure
struct Solution {
    Chromosome chrom; // Chromosome representation
    vector<Route> truckRoutes;
    vector<vector<Route>> droneRoutes; // Multiple trips per drone
    
    // Objectives
    double systemCompletionTime;  // thời gian hoàn thành lấy mẫu cuối cùng
    double totalSampleWaitingTime; // Sum of all waiting times
    
    // Pareto ranking
    int paretoRank;
    double crowdingDistance;
    
    uint64_t solutionHash;
    
    Solution() : systemCompletionTime(INF), totalSampleWaitingTime(INF),
                 paretoRank(0), crowdingDistance(0) {}
    
    // kiểm tra xem có dominate với lời giải khác không
    bool dominates(const Solution& other) const {
        bool better = false;
        
        if (systemCompletionTime <= other.systemCompletionTime &&
            totalSampleWaitingTime <= other.totalSampleWaitingTime) {
            
            if (systemCompletionTime < other.systemCompletionTime ||
                totalSampleWaitingTime < other.totalSampleWaitingTime) {
                better = true;
            }
        }
        return better;
    }
    
    void clear() {
        truckRoutes.clear();
        droneRoutes.clear();
        systemCompletionTime = INF;
        totalSampleWaitingTime = INF;
        paretoRank = 0;
        crowdingDistance = 0;
    }
};


// Individual in population (Empire or Colony)
struct Individual {
    // vector<int> permutation;  // bộ gen hoán vị
    Chromosome chrom;      // Chromosome representation
    Solution solution;             // lời giải sau khi decoded từ hoán vị
    
    Individual() {}
    Individual(Chromosome c) : chrom(c) {}
};


// Empire structure
struct Empire {
    Individual imperialist;
    vector<Individual> colonies;
    double power;
    
    Empire() : power(0) {}
    
    int getTotalSize() const {
        return 1 + colonies.size();
    }
};


#endif // DATASTRUCTURES_H