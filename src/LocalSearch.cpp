#include "LocalSearch.h"
#include <algorithm>
#include <queue>
#include <iostream> 
#include <cmath>
#include <set>


// ========== HELPER FUNCTION: Get Candidate Customers ==========
std::vector<int> LocalSearch::getCandidateCustomers(const Solution& solution, int maxCandidates) {
    std::vector<int> candidates;
    std::set<int> visited;
    
    for (const auto& route : solution.truckRoutes) {
        for (int cust : route.customers) {
            if (visited.find(cust) == visited.end()) {
                candidates.push_back(cust);
                visited.insert(cust);
            }
        }
    }
    
    for (const auto& trips : solution.droneRoutes) {
        for (const auto& trip : trips) {
            for (int cust : trip.customers) {
                if (visited.find(cust) == visited.end()) {
                    candidates.push_back(cust);
                    visited.insert(cust);
                }
            }
        }
    }
    
    if (candidates.size() > static_cast<size_t>(maxCandidates)) {
        candidates.resize(maxCandidates);
    }
    
    return candidates;
}


// ========== MAIN IMPROVE FUNCTION WITH IMPROVEMENTS ==========
Solution LocalSearch::improve(const Solution& solution, int maxIterations) {
    Solution current = solution;
    Solution best = solution;
    
    tabuList.clear();
    int iterWithoutImprovement = 0;
    
    // ⭐ APPLY 4 IMPROVEMENTS BEFORE TABU SEARCH
    balanceDroneLoad(current);
    

    // NEW CODE HERE: Priority 2 - Better load balancing
    std::vector<int> droneLoads;
    for (const auto& trips : current.droneRoutes) {
        int count = 0;
        for (const auto& trip : trips) {
            count += trip.customers.size();
        }
        droneLoads.push_back(count);
    }

    if (!droneLoads.empty()) {
        int maxLoad = *std::max_element(droneLoads.begin(), droneLoads.end());
        int minLoad = *std::min_element(droneLoads.begin(), droneLoads.end());
        
        if (maxLoad - minLoad > 5) {
            int maxIdx = std::max_element(droneLoads.begin(), droneLoads.end()) - droneLoads.begin();
            int minIdx = std::min_element(droneLoads.begin(), droneLoads.end()) - droneLoads.begin();
            auto& fromTrips = current.droneRoutes[maxIdx];
            auto& toTrips = current.droneRoutes[minIdx];
            
            while (!fromTrips.empty() && droneLoads[maxIdx] - droneLoads[minIdx] > 3) {
                int shortestIdx = 0;
                double shortestDist = INF;
                for (size_t i = 0; i < fromTrips.size(); i++) {
                    double dist = 0;
                    int prev = 0;
                    for (int cust : fromTrips[i].customers) {
                        dist += instance.getDistance(prev, cust);
                        prev = cust;
                    }
                    dist += instance.getDistance(prev, 0);
                    
                    if (dist < shortestDist) {
                        shortestDist = dist;
                        shortestIdx = i;
                    }
                }
                
                toTrips.push_back(fromTrips[shortestIdx]);
                fromTrips.erase(fromTrips.begin() + shortestIdx);
                droneLoads[maxIdx] -= toTrips.back().customers.size();
                droneLoads[minIdx] += toTrips.back().customers.size();
            }
        }
    }

    for (auto& route : current.truckRoutes) {
        route.customers = twoOpt(route.customers);
    }
    
    interTruckSwap(current);
    splitLongDroneTrips(current);
    
    evaluator.evaluate(current);
    best = current;
    
    for (int iter = 0; iter < maxIterations; iter++) {
        Move bestMove = findBestMove(current);
        
        if (bestMove.customer1 == -1) {
            break;
        }
        
        Solution neighbor = applyMove(current, bestMove);
        evaluator.evaluate(neighbor);
        
        updateTabuList(bestMove.customer1, static_cast<int>(bestMove.type));
        if (bestMove.customer2 != -1) {
            updateTabuList(bestMove.customer2, static_cast<int>(bestMove.type));
        }
        
        if (neighbor.dominates(best)) {
            best = neighbor;
            iterWithoutImprovement = 0;
        } else {
            iterWithoutImprovement++;
        }
        
        current = neighbor;
        
        if (iterWithoutImprovement > 20) {
            break;
        }
    }
    
    return best;
}


// ========== FIND BEST MOVE FUNCTION - OPTIMIZED ==========
LocalSearch::Move LocalSearch::findBestMove(const Solution& solution) {
    Move bestMove;
    bestMove.deltaCost = INF;
    
    std::vector<int> allCustomers = getCandidateCustomers(solution, 30);
    
    double currentCost = solution.systemCompletionTime * 0.5 + 
                         solution.totalSampleWaitingTime * 0.5;
    double improvementThreshold = currentCost * 0.01;
    
    // ========== 1. Try RELOCATE moves ==========
    for (int cust : allCustomers) {
        if (isTabu(cust, Move::RELOCATE)) continue;
        
        for (size_t truckId = 0; truckId < solution.truckRoutes.size(); truckId++) {
            const auto& route = solution.truckRoutes[truckId];
            std::vector<size_t> positionsToTry = {0, route.customers.size()};
            
            for (size_t pos : positionsToTry) {
                Move move;
                move.type = Move::RELOCATE;
                move.customer1 = cust;
                move.toRoute = truckId;
                move.toPos = pos;
                
                Solution neighbor = applyMove(solution, move);
                evaluator.evaluate(neighbor);
                
                if (neighbor.systemCompletionTime < INF) {
                    double delta = calculateDelta(solution, neighbor);
                    
                    if (delta < -improvementThreshold) {
                        return move;
                    }
                    
                    if (delta < bestMove.deltaCost) {
                        bestMove = move;
                        bestMove.deltaCost = delta;
                    }
                }
            }
        }
        
        const Customer& customer = instance.customers[cust - 1];
        if (!customer.isStaffOnly) {
            for (size_t droneId = 0; droneId < solution.droneRoutes.size(); droneId++) {
                Move move;
                move.type = Move::RELOCATE;
                move.customer1 = cust;
                move.toRoute = droneId + 1000;
                
                Solution neighbor = applyMove(solution, move);
                evaluator.evaluate(neighbor);
                
                if (neighbor.systemCompletionTime < INF) {
                    double delta = calculateDelta(solution, neighbor);
                    
                    if (delta < -improvementThreshold) {
                        return move;
                    }
                    
                    if (delta < bestMove.deltaCost) {
                        bestMove = move;
                        bestMove.deltaCost = delta;
                    }
                }
            }
        }
    }
    
    // ========== 2. Try SWAP moves ==========
    for (size_t i = 0; i < allCustomers.size(); i++) {
        for (size_t j = i + 1; j < allCustomers.size(); j++) {
            int cust1 = allCustomers[i];
            int cust2 = allCustomers[j];
            
            if (isTabu(cust1, Move::SWAP) || isTabu(cust2, Move::SWAP)) continue;
            
            double distance = instance.getDistance(cust1, cust2);
            if (distance < 500) {
                continue;
            }
            
            Move move;
            move.type = Move::SWAP;
            move.customer1 = cust1;
            move.customer2 = cust2;
            
            Solution neighbor = applyMove(solution, move);
            
            bool feasible = true;
            for (const auto& trips : neighbor.droneRoutes) {
                for (const auto& trip : trips) {
                    for (int c : trip.customers) {
                        if (instance.customers[c-1].isStaffOnly) {
                            feasible = false; 
                            break;
                        }
                    }
                    if (!feasible) break;
                }
                if (!feasible) break;
            }
            
            if (!feasible) continue;
            
            evaluator.evaluate(neighbor);
            
            if (neighbor.systemCompletionTime < INF) {
                double delta = calculateDelta(solution, neighbor);
                
                if (delta < -improvementThreshold) {
                    return move;
                }
                
                if (delta < bestMove.deltaCost) {
                    bestMove = move;
                    bestMove.deltaCost = delta;
                }
            }
        }
    }
    
    return bestMove;
}


// ========== APPLY MOVE FUNCTION ==========
Solution LocalSearch::applyMove(const Solution& solution, const Move& move) {
    Solution result = solution;
    
    if (move.type == Move::RELOCATE) {
        bool found = false;
        
        for (auto& route : result.truckRoutes) {
            auto it = std::find(route.customers.begin(), route.customers.end(), move.customer1);
            if (it != route.customers.end()) {
                route.customers.erase(it);
                found = true; 
                break;
            }
        }
        
        if (!found) {
            for (auto& trips : result.droneRoutes) {
                for (auto& trip : trips) {
                    auto it = std::find(trip.customers.begin(), trip.customers.end(), move.customer1);
                    if (it != trip.customers.end()) {
                        trip.customers.erase(it);
                        found = true; 
                        break;
                    }
                }
                if (found) break;
            }
        }
        
        if (move.toRoute < 1000) {
            auto& target = result.truckRoutes[move.toRoute].customers;
            size_t pos = std::min((size_t)move.toPos, target.size());
            target.insert(target.begin() + pos, move.customer1);
        } else {
            int droneId = move.toRoute - 1000;
            Route newTrip;
            newTrip.customers.push_back(move.customer1);
            result.droneRoutes[droneId].push_back(newTrip);
        }
        
    } else if (move.type == Move::SWAP) {
        int* ptr1 = nullptr;
        int* ptr2 = nullptr;
        
        auto findCustomerPtr = [&](int c) -> int* {
            for (auto& route : result.truckRoutes) {
                for (auto& val : route.customers) {
                    if (val == c) return &val;
                }
            }
            for (auto& trips : result.droneRoutes) {
                for (auto& trip : trips) {
                    for (auto& val : trip.customers) {
                        if (val == c) return &val;
                    }
                }
            }
            return nullptr;
        };
        
        ptr1 = findCustomerPtr(move.customer1);
        ptr2 = findCustomerPtr(move.customer2);
        
        if (ptr1 && ptr2) {
            std::swap(*ptr1, *ptr2);
        }
    }
    
    result.systemCompletionTime = 0;
    result.totalSampleWaitingTime = 0;
    
    return result;
}


// ========== TABU LIST MANAGEMENT ==========
bool LocalSearch::isTabu(int customer, int moveType) const {
    return tabuList.find(std::make_pair(customer, moveType)) != tabuList.end();
}


void LocalSearch::updateTabuList(int customer, int moveType) {
    tabuList.insert({customer, moveType});
    
    if (tabuList.size() > static_cast<size_t>(tabuTenure)) {
        tabuList.erase(tabuList.begin());
    }
}


// ========== CALCULATE DELTA COST ==========
double LocalSearch::calculateDelta(const Solution& current, 
                                   const Solution& neighbor) {
    double w1 = 0.5, w2 = 0.5;
    
    double delta1 = neighbor.systemCompletionTime - current.systemCompletionTime;
    double delta2 = neighbor.totalSampleWaitingTime - current.totalSampleWaitingTime;
    
    return w1 * delta1 + w2 * delta2;
}


// ========== IMPROVEMENT 1: DRONE LOAD BALANCING ==========
void LocalSearch::balanceDroneLoad(Solution& solution) {
    std::vector<int> tripCounts;
    for (const auto& trips : solution.droneRoutes) {
        tripCounts.push_back(trips.size());
    }
    
    int maxDrone = 0, minDrone = 0;
    int maxTrips = tripCounts[0], minTrips = tripCounts[0];
    
    for (size_t i = 1; i < tripCounts.size(); i++) {
        if (tripCounts[i] > maxTrips) {
            maxTrips = tripCounts[i];
            maxDrone = i;
        }
        if (tripCounts[i] < minTrips) {
            minTrips = tripCounts[i];
            minDrone = i;
        }
    }
    
    if (maxTrips - minTrips > 3) {
        auto& fromTrips = solution.droneRoutes[maxDrone];
        auto& toTrips = solution.droneRoutes[minDrone];
        
        int shortestIdx = 0;
        double shortestDist = INF;
        
        for (size_t i = 0; i < fromTrips.size(); i++) {
            double dist = 0;
            int prev = 0;
            for (int cust : fromTrips[i].customers) {
                dist += instance.getDistance(prev, cust);
                prev = cust;
            }
            dist += instance.getDistance(prev, 0);
            
            if (dist < shortestDist) {
                shortestDist = dist;
                shortestIdx = i;
            }
        }
        
        toTrips.push_back(fromTrips[shortestIdx]);
        fromTrips.erase(fromTrips.begin() + shortestIdx);
    }
}


// ========== IMPROVEMENT 2: 2-OPT OPTIMIZATION ==========
std::vector<int> LocalSearch::twoOpt(const std::vector<int>& route) {
    if (route.size() < 4) return route;
    
    std::vector<int> best = route;
    double bestDist = calculateRouteDistance(route);
    bool improved = true;
    
    while (improved) {
        improved = false;
        
        for (size_t i = 1; i < best.size() - 2; i++) {
            for (size_t j = i + 1; j < best.size() - 1; j++) {
                std::vector<int> newRoute = best;
                std::reverse(newRoute.begin() + i, newRoute.begin() + j + 1);
                
                double newDist = calculateRouteDistance(newRoute);
                
                if (newDist < bestDist - 1e-6) {
                    best = newRoute;
                    bestDist = newDist;
                    improved = true;
                }
            }
        }
    }
    
    return best;
}

double LocalSearch::calculateRouteDistance(const std::vector<int>& route) {
    double dist = 0;
    int prev = 0;
    
    for (int cust : route) {
        dist += instance.getDistance(prev, cust);
        prev = cust;
    }
    dist += instance.getDistance(prev, 0);
    
    return dist;
}


// ========== IMPROVEMENT 3: INTER-TRUCK SWAP ==========
void LocalSearch::interTruckSwap(Solution& solution) {
    if (solution.truckRoutes.size() < 2) return;
    
    int maxTruck = 0, minTruck = 1;
    int maxLoad = solution.truckRoutes[0].customers.size();
    int minLoad = solution.truckRoutes[1].customers.size();
    
    for (size_t i = 0; i < solution.truckRoutes.size(); i++) {
        int load = solution.truckRoutes[i].customers.size();
        if (load > maxLoad) {
            maxLoad = load;
            maxTruck = i;
        }
        if (load < minLoad) {
            minLoad = load;
            minTruck = i;
        }
    }
    
    if (maxLoad - minLoad > 2) {
        auto& fromRoute = solution.truckRoutes[maxTruck].customers;
        auto& toRoute = solution.truckRoutes[minTruck].customers;
        
        int bestCust = fromRoute[0];
        double minDist = instance.getDistance(0, bestCust);
        
        for (int cust : fromRoute) {
            double dist = instance.getDistance(0, cust);
            if (dist < minDist) {
                minDist = dist;
                bestCust = cust;
            }
        }
        
        fromRoute.erase(std::find(fromRoute.begin(), fromRoute.end(), bestCust));
        toRoute.push_back(bestCust);
    }
}


// ========== IMPROVEMENT 4: SPLIT LONG DRONE TRIPS ==========
void LocalSearch::splitLongDroneTrips(Solution& solution) {
    const double MAX_TRIP_TIME = 3600.0;
    const double cruiseSpeed = 15.65;
    double maxDistPerTrip = MAX_TRIP_TIME * cruiseSpeed * 0.92;  // 51833m
    
    for (size_t droneId = 0; droneId < solution.droneRoutes.size(); droneId++) {
        auto& trips = solution.droneRoutes[droneId];
        std::vector<Route> newTrips;
        
        for (const auto& trip : trips) {
            double dist = 0;
            int prev = 0;
            for (int cust : trip.customers) {
                dist += instance.getDistance(prev, cust);
                prev = cust;
            }
            dist += instance.getDistance(prev, 0);
            
            if (dist > maxDistPerTrip && trip.customers.size() >= 1) {
                int numSplits = (int)std::ceil((double)dist / maxDistPerTrip);
                int customersPerSplit = (int)std::ceil((double)trip.customers.size() / numSplits);
                
                for (int i = 0; i < numSplits; i++) {
                    int start = i * customersPerSplit;
                    int end = std::min(start + customersPerSplit, (int)trip.customers.size());
                    
                    if (start < (int)trip.customers.size()) {
                        Route newTrip;
                        newTrip.customers.assign(trip.customers.begin() + start, 
                                                trip.customers.begin() + end);
                        newTrips.push_back(newTrip);
                    }
                }
            } else {
                newTrips.push_back(trip);
            }
        }
        
        trips = newTrips;
    }
}

