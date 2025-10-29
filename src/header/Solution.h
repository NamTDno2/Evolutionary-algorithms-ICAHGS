#ifndef SOLUTION_H
#define SOLUTION_H

#include "DataStructures.h"

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

#endif // SOLUTION_H
