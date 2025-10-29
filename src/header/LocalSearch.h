#ifndef LOCALSEARCH_H
#define LOCALSEARCH_H

#include "DataStructures.h"
#include "Solution.h"
#include <set>
#include <utility>

class LocalSearch {
public:
    LocalSearch(const Instance& inst) : instance(inst), evaluator(inst) {}
    
    Solution improve(const Solution& solution, int maxIterations = 100);
    
private:
    const Instance& instance;
    SolutionEvaluator evaluator;
    
    // Tabu list: stores (customer_id, move_type) pairs
    std::set<std::pair<int, int>> tabuList;
    int tabuTenure = 7;
    
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
    
    Move findBestMove(const Solution& solution);
    Solution applyMove(const Solution& solution, const Move& move);
    
    bool isTabu(int customer, int moveType) const;
    void updateTabuList(int customer, int moveType);
    
    double calculateDelta(const Solution& current, const Solution& neighbor);
};

#endif // LOCALSEARCH_H
