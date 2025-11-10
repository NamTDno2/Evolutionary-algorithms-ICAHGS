#ifndef DECODER_H
#define DECODER_H

#include "DataStructures.h"
#include "Solution.h"

class Decoder {
public:
    Decoder(const Instance& inst) : instance(inst), evaluator(inst) {}
    
    Solution decode(const std::vector<int>& permutation);
    //  NEW: Incremental decoder
    Solution decodeIncremental(const std::vector<int>& permutation);
    
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
    
    InsertionMove findBestTruckInsertion(int custId, Solution& solution);
    InsertionMove findBestDroneInsertion(int custId, Solution& solution);
    
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
                                      bool newTrip);
    
    // Helper: Evaluate single route (not entire solution)
    double evaluateSingleTruckRoute(const Route& route, int truckId);
    double evaluateSingleDroneTrip(const Route& trip, int droneId);
};

#endif // DECODER_H
