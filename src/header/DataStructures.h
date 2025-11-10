#ifndef DATASTRUCTURES_H
#define DATASTRUCTURES_H

#include <vector>
#include <string>
#include <limits>
#include <cmath> 
#include <cstdint>
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
    double sigma;      // Speed factor
    
    TimeInterval() : startTime(0), endTime(0), sigma(1.0) {}
    TimeInterval(double s, double e, double sig) 
        : startTime(s), endTime(e), sigma(sig) {}
};

// Drone parameters
struct DroneParams {
    double maxCapacity;     // Md (kg)
    double maxEnergy;       // E (kJ)
    double takeoffSpeed;    // m/s
    double cruiseSpeed;     // m/s
    double landingSpeed;    // m/s
    double beta;            // W/kg - energy per unit mass
    double gamma;           // W - base energy consumption
    double maxFlightTime;   // seconds
    
    DroneParams() : maxCapacity(0), maxEnergy(0), takeoffSpeed(0), 
                    cruiseSpeed(0), landingSpeed(0), beta(0), gamma(0),
                    maxFlightTime(0) {}
};

// Truck parameters
struct TruckParams {
    double maxSpeed;  // Vmax (m/s)
    std::vector<TimeInterval> timeIntervals;  // Speed schedule
    
    TruckParams() : maxSpeed(0) {}
};

// Instance data
struct Instance {
    int numTrucks;
    int numDrones;
    std::vector<Customer> customers;
    DroneParams droneParams;
    TruckParams truckParams;
    
    // Depot is always at (0, 0)
    double depotX = 0.0;
    double depotY = 0.0;
    
    int getNumCustomers() const { return customers.size(); }
    
    // Calculate Euclidean distance
    double getDistance(double x1, double y1, double x2, double y2) const {
        double dx = x2 - x1;
        double dy = y2 - y1;
        return sqrt(dx * dx + dy * dy);
    }
    
    double getDistance(int custId1, int custId2) const {
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
};

// Route structure
struct Route {
    std::vector<int> customers;  // Customer IDs (0 = depot)
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
    std::vector<Route> truckRoutes;
    std::vector<std::vector<Route>> droneRoutes; // Multiple trips per drone
    
    // Objectives
    double systemCompletionTime;  // Max completion time
    double totalSampleWaitingTime; // Sum of all waiting times
    
    // Pareto ranking
    int paretoRank;
    double crowdingDistance;
    
    uint64_t solutionHash;
    
    Solution() : systemCompletionTime(INF), totalSampleWaitingTime(INF),
                 paretoRank(0), crowdingDistance(0) {}
    
    // Pareto dominance check
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
    std::vector<int> permutation;  // Customer permutation
    Solution solution;             // Decoded solution
    
    Individual() {}
    Individual(int n) : permutation(n) {
        for (int i = 0; i < n; i++) {
            permutation[i] = i + 1;  // Customer IDs start from 1
        }
    }
};

// Empire structure
struct Empire {
    Individual imperialist;
    std::vector<Individual> colonies;
    double power;
    
    Empire() : power(0) {}
    
    int getTotalSize() const {
        return 1 + colonies.size();
    }
};

#endif // DATASTRUCTURES_H
