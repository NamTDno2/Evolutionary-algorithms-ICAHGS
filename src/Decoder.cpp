#include "Decoder.h"
#include <algorithm>
#include <limits>

Solution Decoder::decode(const std::vector<int>& permutation) {
    Solution solution;
    std::vector<bool> servedCustomers(instance.getNumCustomers() + 1, false);
    
    // Initialize routes
    solution.truckRoutes.resize(instance.numTrucks);
    solution.droneRoutes.resize(instance.numDrones);
    
    // Process each customer in permutation order
    for (int custId : permutation) {
        if (servedCustomers[custId]) continue; // Bỏ qua nếu đã phục vụ

        // Evaluate the current partial solution before finding the best insertion
        evaluator.evaluate(solution);

        const Customer& cust = instance.customers[custId - 1];
        
        InsertionMove bestMove;
        
        if (cust.isStaffOnly) {
            // Must use truck
            bestMove = findBestTruckInsertion(custId, solution);
        } else {
            // Try both truck and drone
            InsertionMove truckMove = findBestTruckInsertion(custId, solution);
            InsertionMove droneMove = findBestDroneInsertion(custId, solution);
            
            bestMove = (truckMove.cost < droneMove.cost) ? truckMove : droneMove;
        }
        
        // Apply best insertion
        if (bestMove.routeType == 0) {
            // Insert into truck route
            auto& route = solution.truckRoutes[bestMove.routeId];
            route.customers.insert(route.customers.begin() + bestMove.position, 
                                 custId);
        } else if (bestMove.routeType == 1) {
            // Insert into drone route
            auto& droneTrips = solution.droneRoutes[bestMove.routeId];
            
            if (bestMove.position >= droneTrips.size()) { // Sửa logic: dùng position thay vì routeId
                // Create new trip
                Route newTrip;
                newTrip.customers.push_back(custId);
                droneTrips.push_back(newTrip);
            } else {
                // Insert into existing trip
                droneTrips[bestMove.position].customers.push_back(custId);
            }
        }
        
        servedCustomers[custId] = true; // Đánh dấu đã phục vụ
    }
    
    // Evaluate final solution
    evaluator.evaluate(solution);
    
    return solution;
}

Decoder::InsertionMove Decoder::findBestTruckInsertion(int custId, Solution& solution) {
    InsertionMove bestMove;
    bestMove.routeType = 0;
    
    // Lưu top-K moves tốt nhất
    std::vector<InsertionMove> topMoves;
    
    for (int truckId = 0; truckId < instance.numTrucks; truckId++) {
        auto& route = solution.truckRoutes[truckId];
        
        for (size_t pos = 0; pos <= route.customers.size(); pos++) {
            Solution tempSolution = solution;
            tempSolution.truckRoutes[truckId].customers.insert(
                tempSolution.truckRoutes[truckId].customers.begin() + pos, 
                custId);
            
            evaluator.evaluate(tempSolution);
            
            if (tempSolution.systemCompletionTime < INF) {
                double cost = evaluateInsertionCost(solution, tempSolution);
                
                InsertionMove move;
                move.routeType = 0;
                move.routeId = truckId;
                move.position = pos;
                move.cost = cost;
                
                topMoves.push_back(move);
            }
        }
    }
    
    if (topMoves.empty()) {
        return bestMove;
    }
    
    // Sort by cost
    std::sort(topMoves.begin(), topMoves.end(), 
              [](const InsertionMove& a, const InsertionMove& b) {
        return a.cost < b.cost;
    });
    
    // **THÊM RANDOMNESS: Chọn từ top-3 thay vì best**
    int selectRange = std::min(3, (int)topMoves.size());
    int selectedIdx = rand() % selectRange;
    
    return topMoves[selectedIdx];
}


Decoder::InsertionMove Decoder::findBestDroneInsertion(int custId, Solution& solution) {
    InsertionMove bestMove;
    bestMove.routeType = 1;
    
    const Customer& cust = instance.customers[custId - 1];
    
    // Lưu top-K moves tốt nhất
    std::vector<InsertionMove> topMoves;
    
    for (int droneId = 0; droneId < instance.numDrones; droneId++) {
        auto& trips = solution.droneRoutes[droneId];
        
        // === Option 1: Thêm vào existing trip ===
        for (size_t tripId = 0; tripId < trips.size(); tripId++) {
            // Check capacity
            double currentLoad = 0;
            for (int c : trips[tripId].customers) {
                currentLoad += instance.customers[c - 1].demand;
            }
            
            if (currentLoad + cust.demand > instance.droneParams.maxCapacity) {
                continue;  // Skip nếu vượt capacity
            }
            
            // Try insertion
            Solution tempSolution = solution;
            tempSolution.droneRoutes[droneId][tripId].customers.push_back(custId);
            
            evaluator.evaluate(tempSolution);
            
            if (tempSolution.systemCompletionTime < INF) {
                double cost = evaluateInsertionCost(solution, tempSolution);
                
                InsertionMove move;
                move.routeType = 1;
                move.routeId = droneId;
                move.position = tripId;  // Existing trip
                move.cost = cost;
                
                topMoves.push_back(move);
            }
        }
        
        // === Option 2: Create new trip (direct flight) ===
        Solution tempSolution = solution;
        Route newTrip;
        newTrip.customers.push_back(custId);
        tempSolution.droneRoutes[droneId].push_back(newTrip);
        
        evaluator.evaluate(tempSolution);
        
        if (tempSolution.systemCompletionTime < INF) {
            double cost = evaluateInsertionCost(solution, tempSolution);
            
            InsertionMove move;
            move.routeType = 1;
            move.routeId = droneId;
            move.position = trips.size();  // New trip index
            move.cost = cost;
            
            topMoves.push_back(move);
        }
    }
    
    if (topMoves.empty()) {
        return bestMove;  // No feasible drone insertion
    }
    
    // Sort by cost (ascending)
    std::sort(topMoves.begin(), topMoves.end(), 
              [](const InsertionMove& a, const InsertionMove& b) {
        return a.cost < b.cost;
    });
    
    // **THÊM RANDOMNESS: Chọn từ top-3 thay vì best**
    int selectRange = std::min(3, (int)topMoves.size());
    int selectedIdx = rand() % selectRange;
    
    return topMoves[selectedIdx];
}


double Decoder::evaluateInsertionCost(const Solution& before, 
                                     const Solution& after) {
    // Simple weighted sum for now (can be improved with scalarization)
    double w1 = 0.5, w2 = 0.5;
    
    double deltaCompletion = after.systemCompletionTime - 
                            before.systemCompletionTime;
    double deltaWaiting = after.totalSampleWaitingTime - 
                         before.totalSampleWaitingTime;
    
    return w1 * deltaCompletion + w2 * deltaWaiting;
}
