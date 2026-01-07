#ifndef LOCALSEARCH_H
#define LOCALSEARCH_H

#include "DataStructures.h"
#include "Solution.h"
#include <set>

class LocalSearch {
public:
    LocalSearch(const Instance& inst) : instance(inst), evaluator(inst) {}
    
    Solution improve(const Solution& solution, int maxIterations = 100);
    
private:
    const Instance& instance;
    SolutionEvaluator evaluator;
    
    // Tabu list: stores (customer_id, move_type) pairs
    std::set<std::pair<int, int>> tabuList;
    int tabuTenure = 10;  // Benchmark standard value
    
    struct Move {
        enum Type { RELOCATE, SWAP, SWAP_STAR };
        Type type;
        int customer1, customer2;
        int fromRoute, toRoute;
        int fromPos, toPos;
        double deltaCost;
        
        Move() : type(RELOCATE), customer1(-1), customer2(-1),
                fromRoute(-1), toRoute(-1), fromPos(-1), toPos(-1),
                deltaCost(INF) {}
    };
    
    // ========== OPTIMIZED FUNCTIONS ==========
    std::vector<int> getCandidateCustomers(const Solution& solution, int maxCandidates = 30);
    Move findBestMove(const Solution& solution);
    Solution applyMove(const Solution& solution, const Move& move);
    bool isTabu(int customer, int moveType) const;
    void updateTabuList(int customer, int moveType);
    double calculateDelta(const Solution& current, const Solution& neighbor);
    
    // ========== IMPROVEMENT FUNCTIONS ==========
    // Improvement 1: Balance drone load
    void balanceDroneLoad(Solution& solution);
    
    // Improvement 2: 2-opt for truck routes
    std::vector<int> twoOpt(const std::vector<int>& route);
    double calculateRouteDistance(const std::vector<int>& route);
    
    // NEW: Improvement 2b: 2-opt directly on solution
    void twoOptTrucks(Solution& solution);
    
    // NEW: Or-opt operator for trucks (move sequence of 1-3 customers)
    void orOptTrucks(Solution& solution);
    
    // NEW: Improvement 5: Smart drone-truck transfer
    void smartDroneTruckTransfer(Solution& solution);
    
    // NEW: Improvement 6: Optimize drone trip sequences
    void optimizeDroneTrips(Solution& solution);
    
    // NEW: GENI (Generalized Insertion) - Remove and reinsert at best position
    void geniOperator(Solution& solution, int maxIterations = 50);
    
    // Improvement 3: Inter-truck swap
    void interTruckSwap(Solution& solution);
    
    // Improvement 4: Split long drone trips
    void splitLongDroneTrips(Solution& solution);
};

#endif // LOCALSEARCH_H