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
            bestMove = findBestTruckInsertionIncremental(custId, solution);
        } else {
            InsertionMove truckMove = findBestTruckInsertionIncremental(custId, solution);
            InsertionMove droneMove = findBestDroneInsertionIncremental(custId, solution);
            
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

Decoder::InsertionMove Decoder::findBestTruckInsertionIncremental(
    int custId, 
    Solution& solution) {
    
    InsertionMove bestMove;
    bestMove.routeType = 0;
    bestMove.cost = INF;
    
    // Thử tất cả trucks và positions
    for (int truckId = 0; truckId < instance.numTrucks; truckId++) {
        auto& route = solution.truckRoutes[truckId];
        
        for (size_t pos = 0; pos <= route.customers.size(); pos++) {
            // ⭐ Tính delta cost thay vì evaluate toàn bộ
            double deltaCost = computeTruckInsertionDelta(solution, custId, truckId, pos);
            
            if (deltaCost < bestMove.cost) {
                bestMove.cost = deltaCost;
                bestMove.routeId = truckId;
                bestMove.position = pos;
            }
        }
    }
    
    return bestMove;
}



Decoder::InsertionMove Decoder::findBestDroneInsertionIncremental(
    int custId,
    Solution& solution) {
    
    InsertionMove bestMove;
    bestMove.routeType = 1;
    bestMove.cost = INF;
    
    const Customer& cust = instance.customers[custId - 1];
    
    for (int droneId = 0; droneId < instance.numDrones; droneId++) {
        auto& trips = solution.droneRoutes[droneId];
        
        // ========== Option 1: Add to existing trip ==========
        for (size_t tripId = 0; tripId < trips.size(); tripId++) {
            // Check capacity
            double currentLoad = 0;
            for (int c : trips[tripId].customers) {
                currentLoad += instance.customers[c - 1].demand;
            }
            
            if (currentLoad + cust.demand > instance.droneParams.maxCapacity) {
                continue;  // Skip
            }
            
            // Tính delta cost
            double deltaCost = computeDroneInsertionDelta(
                solution, custId, droneId, tripId, false
            );
            
            if (deltaCost < bestMove.cost) {
                bestMove.cost = deltaCost;
                bestMove.routeId = droneId;
                bestMove.position = tripId;
            }
        }
        
        // ========== Option 2: Create new trip ==========
        double deltaCost = computeDroneInsertionDelta(
            solution, custId, droneId, trips.size(), true
        );
        
        if (deltaCost < bestMove.cost) {
            bestMove.cost = deltaCost;
            bestMove.routeId = droneId;
            bestMove.position = trips.size();
        }
    }
    
    return bestMove;
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

// ==================== INCREMENTAL DECODER ====================

Solution Decoder::decodeIncremental(const std::vector<int>& permutation) {
    Solution solution;
    solution.truckRoutes.resize(instance.numTrucks);
    solution.droneRoutes.resize(instance.numDrones);
    
    // Initialize với objectives = 0
    solution.systemCompletionTime = 0;
    solution.totalSampleWaitingTime = 0;
    
    std::vector<bool> servedCustomers(instance.getNumCustomers() + 1, false);
    
    for (int custId : permutation) {
        if (servedCustomers[custId]) continue;
        
        const Customer& cust = instance.customers[custId - 1];
        
        InsertionMove bestMove;
        
        if (cust.isStaffOnly) {
            // Must use truck
            bestMove = findBestTruckInsertionIncremental(custId, solution);
        } else {
            // Try both
            InsertionMove truckMove = findBestTruckInsertionIncremental(custId, solution);
            InsertionMove droneMove = findBestDroneInsertionIncremental(custId, solution);
            
            bestMove = (truckMove.cost < droneMove.cost) ? truckMove : droneMove;
        }
        
        // Apply move
        if (bestMove.routeType == 0) {
            // Truck
            solution.truckRoutes[bestMove.routeId].customers.insert(
                solution.truckRoutes[bestMove.routeId].customers.begin() + bestMove.position,
                custId
            );
        } else {
            // Drone
            if (bestMove.position < (int)solution.droneRoutes[bestMove.routeId].size()) {
                // Existing trip
                solution.droneRoutes[bestMove.routeId][bestMove.position].customers.push_back(custId);
            } else {
                // New trip
                Route newTrip;
                newTrip.customers.push_back(custId);
                solution.droneRoutes[bestMove.routeId].push_back(newTrip);
            }
        }
        
        servedCustomers[custId] = true;
    }
    
    // ⭐ Final evaluation một lần duy nhất
    evaluator.evaluate(solution);
    
    return solution;
}
double Decoder::computeTruckInsertionDelta(
    const Solution& current,
    int custId,
    int truckId,
    int position) {
    
    const auto& route = current.truckRoutes[truckId];
    const Customer& newCust = instance.customers[custId - 1];
    
    // ========== Tính OLD COST (trước khi thêm) ==========
    double oldCompletionTime = 0;
    double oldWaitingTime = 0;
    
    if (!route.customers.empty()) {
        // Route đã có customers
        double currentTime = 0;
        
        // Depot → First customer
        double distance = instance.getDistance(0, route.customers[0]);
        double travelTime = distance / instance.truckParams.maxSpeed;
        currentTime += travelTime;
        
        // Service first customer
        currentTime += instance.customers[route.customers[0] - 1].serviceTimeTruck;
        
        // Traverse route
        for (size_t i = 1; i < route.customers.size(); i++) {
            int prev = route.customers[i-1];
            int curr = route.customers[i];
            
            distance = instance.getDistance(prev, curr);
            travelTime = distance / instance.truckParams.maxSpeed;
            currentTime += travelTime;
            
            // Service
            currentTime += instance.customers[curr - 1].serviceTimeTruck;
            
            // Waiting time
            oldWaitingTime += currentTime;
        }
        
        // Return to depot
        distance = instance.getDistance(route.customers.back(), 0);
        travelTime = distance / instance.truckParams.maxSpeed;
        currentTime += travelTime;
        
        oldCompletionTime = currentTime;
    }
    
    // ========== Tính NEW COST (sau khi thêm) ==========
    double newCompletionTime = 0;
    double newWaitingTime = 0;
    
    // Tạo route tạm thời
    std::vector<int> newRoute = route.customers;
    newRoute.insert(newRoute.begin() + position, custId);
    
    double currentTime = 0;
    
    // Depot → First customer
    double distance = instance.getDistance(0, newRoute[0]);
    double travelTime = distance / instance.truckParams.maxSpeed;
    currentTime += travelTime;
    
    // Service first
    currentTime += instance.customers[newRoute[0] - 1].serviceTimeTruck;
    
    // Traverse
    for (size_t i = 1; i < newRoute.size(); i++) {
        int prev = newRoute[i-1];
        int curr = newRoute[i];
        
        distance = instance.getDistance(prev, curr);
        travelTime = distance / instance.truckParams.maxSpeed;
        currentTime += travelTime;
        
        // Service
        currentTime += instance.customers[curr - 1].serviceTimeTruck;
        
        // Waiting
        newWaitingTime += currentTime;
    }
    
    // Return
    distance = instance.getDistance(newRoute.back(), 0);
    travelTime = distance / instance.truckParams.maxSpeed;
    currentTime += travelTime;
    
    newCompletionTime = currentTime;
    
    // ========== DELTA COST ==========
    double deltaCT = newCompletionTime - oldCompletionTime;
    double deltaWT = newWaitingTime - oldWaitingTime;
    
    // Weighted sum (có thể điều chỉnh weights)
    double deltaCost = 0.5 * deltaCT + 0.5 * deltaWT;
    
    return deltaCost;
}
double Decoder::computeDroneInsertionDelta(
    const Solution& current,
    int custId,
    int droneId,
    int tripId,
    bool newTrip) {
    
    const Customer& newCust = instance.customers[custId - 1];
    
    // ========== OLD COST ==========
    double oldCompletionTime = 0;
    double oldWaitingTime = 0;
    
    // ========== OLD COST ========== (Dòng ~285)
if (!newTrip && tripId < (int)current.droneRoutes[droneId].size()) {
    const auto& trip = current.droneRoutes[droneId][tripId];
    
    if (!trip.customers.empty()) {
        double currentTime = 0;
        double currentLoad = 0;
        
        // Depot → customers → Depot
        for (size_t i = 0; i < trip.customers.size(); i++) {
            int custId_old = trip.customers[i];
            const Customer& c = instance.customers[custId_old - 1];
            
            currentLoad += c.demand;
            
            // Travel time
            double distance = instance.getDistance(
                (i == 0) ? 0 : trip.customers[i-1], 
                custId_old
            );
            
            // ✅ FIX: Dùng emptySpeed thay vì maxSpeed
            double speed = instance.droneParams.cruiseSpeed;
            currentTime += distance / speed;
            
            // Service
            currentTime += c.serviceTimeDrone;
            
            // Waiting
            oldWaitingTime += currentTime;
        }
        
        // Return
        double returnDist = instance.getDistance(trip.customers.back(), 0);
        double speed = instance.droneParams.cruiseSpeed;
        currentTime += returnDist / speed;
        
        oldCompletionTime = currentTime;
    }
}

    
    // ========== NEW COST ==========
    double newCompletionTime = 0;
    double newWaitingTime = 0;
    
    std::vector<int> newTripCustomers;
    
    if (newTrip) {
        // New trip: chỉ có custId
        newTripCustomers = {custId};
    } else {
        // Existing trip: thêm custId vào cuối
        newTripCustomers = current.droneRoutes[droneId][tripId].customers;
        newTripCustomers.push_back(custId);
    }
    
    // ========== NEW COST ========== (Dòng ~325)
double currentTime = 0;

for (size_t i = 0; i < newTripCustomers.size(); i++) {
    int cid = newTripCustomers[i];
    const Customer& c = instance.customers[cid - 1];
    
    // Travel
    double distance = instance.getDistance(
        (i == 0) ? 0 : newTripCustomers[i-1],
        cid
    );
    
    // ✅ FIX: Dùng emptySpeed
    double speed = instance.droneParams.cruiseSpeed;
    currentTime += distance / speed;
    
    // Service
    currentTime += c.serviceTimeDrone;
    
    // Waiting
    newWaitingTime += currentTime;
}

// Return
double returnDist = instance.getDistance(newTripCustomers.back(), 0);
double speed = instance.droneParams.cruiseSpeed;
currentTime += returnDist / speed;

newCompletionTime = currentTime;

    
    // ========== Check Energy Constraint ==========
    // Simple energy check (có thể refine)
    double totalEnergy = 0;
    double currentLoad = 0;
    
    for (int cid : newTripCustomers) {
        currentLoad += instance.customers[cid - 1].demand;
    }
    
    // Power = beta * load + gamma
    double avgPower = instance.droneParams.beta * (currentLoad / 2) + 
                     instance.droneParams.gamma;
    totalEnergy = avgPower * newCompletionTime;
    
    if (totalEnergy > instance.droneParams.maxEnergy) {
        return INF;  // Infeasible
    }
    
    // ========== DELTA COST ==========
    double deltaCT = newCompletionTime - oldCompletionTime;
    double deltaWT = newWaitingTime - oldWaitingTime;
    
    double deltaCost = 0.5 * deltaCT + 0.5 * deltaWT;
    
    return deltaCost;
}


