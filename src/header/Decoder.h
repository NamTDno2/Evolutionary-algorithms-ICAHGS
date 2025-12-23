#ifndef DECODER_H
#define DECODER_H

#include "DataStructures.h"
#include "Solution.h"
#include "Chromosome.h"

class Decoder {
public:
    Decoder(const Instance& inst) : instance(inst), evaluator(inst) {}
    
    Solution decode(const std::vector<int>& permutation);
    //  NEW: Incremental decoder
    Solution decodeIncremental(const std::vector<int>& permutation);
    //  NEW: Decode from Chromosome
    Solution decode(const Chromosome& chrom, const Instance& inst);
    //  NEW: Split Algorithm - Dynamic Programming based decoder
    Solution decodeBySplit(const Chromosome& chrom);
    
private:
    const Instance& instance;
    SolutionEvaluator evaluator;
    
    struct InsertionMove {
        int routeType;  // 0 = truck, 1 = drone
        int routeId;
        int position;
        double cost;
        
        InsertionMove() : routeType(-1), routeId(-1), position(-1), 
                         cost(INF) {}
    };
    
    
    double evaluateInsertionCost(const Solution& before, 
                                 const Solution& after);
    
    // NEW: Incremental evaluation functions
    InsertionMove findBestTruckInsertionIncremental(int custId, Solution& solution);
    InsertionMove findBestDroneInsertionIncremental(int custId, Solution& solution);
    
    // Helper: Tính delta cost cho truck insertion
    double computeTruckInsertionDelta(const Solution& current, 
                                      int custId, 
                                      int truckId, 
                                      int position);
    
    // Helper: Tính delta cost cho drone insertion
    double computeDroneInsertionDelta(const Solution& current, 
                                      int custId, 
                                      int droneId, 
                                      int tripId,
                                      int position);
    
    // Split Algorithm helpers - DISABLED (causes infeasibility)
    // struct SplitLabel {
    //     double cost;
    //     int pred;
    //     int vehicleType;  // 0=truck, 1=drone
    //     int vehicleId;
    //     SplitLabel() : cost(INF), pred(-1), vehicleType(-1), vehicleId(-1) {}
    // };
    
    // double computeRouteCost(const std::vector<int>& customers, int vehicleType, int vehicleId);
    // bool isRouteFeasible(const std::vector<int>& customers, int vehicleType, int vehicleId);
    
    // Helper: Evaluate single route (not entire solution)
    double evaluateSingleTruckRoute(const Route& route, int truckId);
    double evaluateSingleDroneTrip(const Route& trip, int droneId);
    double calculateTripFlightTime(const Route& trip, int droneId);
};

#endif // DECODER_H
