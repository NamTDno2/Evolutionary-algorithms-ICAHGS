#ifndef DECODER_H
#define DECODER_H

#include "DataStructures.h"
#include "Solution.h"

class Decoder {
public:
    Decoder(const Instance& inst) : instance(inst), evaluator(inst) {}
    
    Solution decode(const std::vector<int>& permutation);
    
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
};

#endif // DECODER_H
