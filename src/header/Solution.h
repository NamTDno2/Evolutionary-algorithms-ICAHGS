#ifndef SOLUTION_H
#define SOLUTION_H

#include "DataStructures.h"
#include <map>         // ← THÊM (cho std::map)
#include <tuple>       // ← THÊM (cho std::tuple)
#include <random>      // ← THÊM (cho std::mt19937_64)
#include <cstdint>

class SolutionEvaluator {
public:
    SolutionEvaluator(const Instance& inst) : instance(inst) {}
    
    void evaluate(Solution& solution);
    
private:
    const Instance& instance;
    
    // Evaluate truck route considering time-dependent speed
    void evaluateTruckRoute(Route& route, int truckId);
    
    // Evaluate drone route considering energy constraints
    bool evaluateDroneRoute(Route& route, int droneId);
    
    // Calculate travel time with time-dependent speed
    double calculateTruckTravelTime(double startTime, double distance);
    
    // Calculate drone energy consumption
    double calculateDroneEnergy(const Route& route);
    
    // Get speed factor at given time
    double getSpeedFactor(double time) const;
};

// Pareto ranking utilities
class ParetoRanking {
public:
    static void nonDominatedSorting(std::vector<Solution*>& solutions);
    static void calculateCrowdingDistance(std::vector<Solution*>& front);
};

// **THÊM MỚI: Zobrist Hashing để detect duplicates**
class SolutionHasher {
public:
    SolutionHasher(int maxCustomers, int maxTrucks, int maxDrones);
    
    // Tính hash của solution
    uint64_t computeHash(const Solution& solution) const;
    
private:
    // Zobrist table: [customer_id][route_id][position] → random number
    std::map<std::tuple<int, int, int>, uint64_t> zobristTable;
    
    void initializeZobristTable(int maxCustomers, int maxTrucks, int maxDrones);
};

#endif // SOLUTION_H
