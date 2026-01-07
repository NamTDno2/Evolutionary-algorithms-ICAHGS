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
            //  VALIDATE: Only add valid customer IDs
            if (cust >= 1 && cust <= (int)instance.customers.size() && 
                visited.find(cust) == visited.end()) {
                candidates.push_back(cust);
                visited.insert(cust);
            }
        }
    }
    
    for (const auto& trips : solution.droneRoutes) {
        for (const auto& trip : trips) {
            for (int cust : trip.customers) {
                //  VALIDATE: Only add valid customer IDs
                if (cust >= 1 && cust <= (int)instance.customers.size() && 
                    visited.find(cust) == visited.end()) {
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
    
    //  CẢI TIẾN: Operators tùy theo kích thước bài toán
    int numCustomers = instance.getNumCustomers();
    
    // 1. Balance drone load - NHANH, hiệu quả cao (luôn dùng)
    balanceDroneLoad(current);
    
    // 2. 2-opt for trucks - Chậm nhưng HIỆU QUẢ RẤT CAO (luôn dùng)
    twoOptTrucks(current);
    
    // 3. Smart drone-truck transfer - NHANH, hiệu quả (luôn dùng)
    smartDroneTruckTransfer(current);
    
    // 4. Inter-truck swap - NHANH, cân bằng tải (luôn dùng)
    interTruckSwap(current);
    
    // 5. RE-ENABLE cho datasets 100+ (chúng chậm nhưng CẦN cho chất lượng)
    if (numCustomers >= 50) {
        orOptTrucks(current);  // Move sequences - quan trọng cho 100+
    }
    
    if (numCustomers >= 100) {
        optimizeDroneTrips(current);  // 2-opt drone - CỰC quan trọng cho 100+
    }
    
    evaluator.evaluate(current);
    best = current;
    
    //  Early stopping - dừng sớm nếu không cải thiện
    int noImprovementLimit = maxIterations / 3;
    
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
            current = neighbor;
            iterWithoutImprovement++;
        }
        
        // CẢI TIẾN: Early stopping khi không cải thiện lâu
        if (iterWithoutImprovement >= noImprovementLimit) {
            break;
        }
    }
    
    // Final cleanup: split/move any remaining long drone trips
    splitLongDroneTrips(best);
    // OPTIMIZATION: Only evaluate if splitLongDroneTrips made changes
    // Since splitLongDroneTrips modifies routes, we need this evaluation
    evaluator.evaluate(best);
    
    return best;
}


// ========== FIND BEST MOVE FUNCTION - OPTIMIZED ==========
LocalSearch::Move LocalSearch::findBestMove(const Solution& solution) {
    Move bestMove;
    bestMove.deltaCost = INF;
    
    std::vector<int> allCustomers = getCandidateCustomers(solution, 70);
    
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
        
        //  VALIDATE: cust must be valid before accessing customers array
        if (cust < 1 || cust > (int)instance.customers.size()) {
            continue;  // Skip invalid customer IDs
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
                        //  VALIDATE: c must be valid
                        if (c < 1 || c > (int)instance.customers.size()) {
                            feasible = false;
                            break;
                        }
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
    // CẢI TIẾN: Adaptive weights dựa trên số lượng trucks
    // High-truck datasets (30+): ưu tiên WT MỌI LÚC (w2=0.85)
    // Low-truck datasets: cân bằng hơn (w2=0.7)
    double w1, w2;
    
    int numTrucks = instance.numTrucks;
    if (numTrucks >= 30) {
        w1 = 0.15;
        w2 = 0.85;  
    } else if (numTrucks >= 20) {
        w1 = 0.20;
        w2 = 0.80;
    } else {
        w1 = 0.3;
        w2 = 0.7;
    }
    
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
    const int height = 50;
    
    for (size_t droneId = 0; droneId < solution.droneRoutes.size(); droneId++) {
        const auto& droneParam = instance.droneParams[droneId];
        const double MAX_TRIP_TIME = droneParam.maxFlightTime;
        const double takeoffSpeed = droneParam.takeoffSpeed;
        const double landingSpeed = droneParam.landingSpeed;
        const double cruiseSpeed = droneParam.cruiseSpeed;
        
        const double takeoffTime = takeoffSpeed != 0 ? height / takeoffSpeed : 0;
        const double landingTime = landingSpeed != 0 ? height / landingSpeed : 0;
        
        auto& trips = solution.droneRoutes[droneId];
        std::vector<Route> newTrips;
        
        for (const auto& trip : trips) {
            // Tính thời gian bay thực tế
            double totalTime = 0;
            int prev = 0;
            for (int cust : trip.customers) {
                double dist = instance.getDistance(prev, cust);
                double cruiseTime = dist / cruiseSpeed;
                totalTime += takeoffTime + cruiseTime + landingTime;
                prev = cust;
            }
            // Quay về depot
            double dist = instance.getDistance(prev, 0);
            totalTime += takeoffTime + (dist / cruiseSpeed) + landingTime;
            
            if (totalTime > MAX_TRIP_TIME && trip.customers.size() > 1) {
                // Split thành nhiều trips nhỏ hơn
                int numSplits = (int)std::ceil(totalTime / MAX_TRIP_TIME);
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
            } else if (totalTime > MAX_TRIP_TIME && trip.customers.size() == 1) {
                // Trip 1 customer nhưng vẫn quá dài - không thể dùng drone
                // Chuyển sang truck (tìm truck có ít customer nhất)
                int bestTruck = 0;
                size_t minSize = solution.truckRoutes[0].customers.size();
                for (size_t t = 1; t < solution.truckRoutes.size(); t++) {
                    if (solution.truckRoutes[t].customers.size() < minSize) {
                        minSize = solution.truckRoutes[t].customers.size();
                        bestTruck = t;
                    }
                }
                solution.truckRoutes[bestTruck].customers.push_back(trip.customers[0]);
                // Không add vào newTrips - loại bỏ khỏi drone
            } else {
                newTrips.push_back(trip);
            }
        }
        
        trips = newTrips;
    }
}

// ========== NEW IMPROVEMENT: 2-OPT FOR TRUCKS (IN-PLACE) ==========
void LocalSearch::twoOptTrucks(Solution& solution) {
    bool globalImproved = true;
    
    while (globalImproved) {
        globalImproved = false;
        
        for (auto& route : solution.truckRoutes) {
            if (route.customers.size() < 4) continue;
            
            bool routeImproved = true;
            while (routeImproved) {
                routeImproved = false;
                int n = route.customers.size();
                
                for (int i = 0; i < n - 2; i++) {
                    for (int j = i + 2; j < n; j++) {
                        // Calculate current distance
                        int prev_i = (i == 0) ? 0 : route.customers[i - 1];
                        int curr_i = route.customers[i];
                        int curr_j = route.customers[j];
                        int next_j = (j == n - 1) ? 0 : route.customers[j + 1];
                        
                        double oldDist = instance.getDistance(prev_i, curr_i) 
                                       + instance.getDistance(curr_j, next_j);
                        double newDist = instance.getDistance(prev_i, curr_j) 
                                       + instance.getDistance(curr_i, next_j);
                        
                        if (newDist < oldDist - 0.01) {
                            // Reverse segment [i, j]
                            std::reverse(route.customers.begin() + i, 
                                       route.customers.begin() + j + 1);
                            routeImproved = true;
                            globalImproved = true;
                        }
                    }
                    if (routeImproved) break;
                }
            }
        }
    }
}

// ========== NEW IMPROVEMENT: OR-OPT FOR TRUCKS ==========
void LocalSearch::orOptTrucks(Solution& solution) {
    // Or-opt: Move a sequence of 1, 2, or 3 consecutive customers to another position
    bool improved = true;
    
    while (improved) {
        improved = false;
        
        for (auto& route : solution.truckRoutes) {
            if (route.customers.size() < 2) continue;
            
            // Try sequence lengths 1, 2, 3
            for (int seqLen = 1; seqLen <= 3 && seqLen <= (int)route.customers.size(); seqLen++) {
                for (int i = 0; i <= (int)route.customers.size() - seqLen; i++) {
                    // Extract sequence [i, i+seqLen)
                    std::vector<int> sequence(route.customers.begin() + i, 
                                            route.customers.begin() + i + seqLen);
                    
                    // Calculate cost of removing sequence
                    int prevNode = (i == 0) ? 0 : route.customers[i - 1];
                    int nextNode = (i + seqLen >= (int)route.customers.size()) ? 
                                   0 : route.customers[i + seqLen];
                    
                    double removeCost = -instance.getDistance(prevNode, sequence[0])
                                      - instance.getDistance(sequence.back(), nextNode)
                                      + instance.getDistance(prevNode, nextNode);
                    
                    // Try inserting at all other positions
                    for (int j = 0; j <= (int)route.customers.size() - seqLen; j++) {
                        if (abs(j - i) <= seqLen) continue; // Skip nearby positions
                        
                        // Calculate insertion cost
                        int insertPrev = (j == 0) ? 0 : route.customers[j - 1];
                        int insertNext = (j >= (int)route.customers.size()) ? 
                                        0 : route.customers[j];
                        
                        double insertCost = instance.getDistance(insertPrev, sequence[0])
                                         + instance.getDistance(sequence.back(), insertNext)
                                         - instance.getDistance(insertPrev, insertNext);
                        
                        if (removeCost + insertCost < -0.01) {
                            // Apply move
                            route.customers.erase(route.customers.begin() + i,
                                                route.customers.begin() + i + seqLen);
                            
                            int insertPos = (j > i) ? j - seqLen : j;
                            route.customers.insert(route.customers.begin() + insertPos,
                                                 sequence.begin(), sequence.end());
                            
                            improved = true;
                            break;
                        }
                    }
                    if (improved) break;
                }
                if (improved) break;
            }
            if (improved) break;
        }
    }
}

// ========== NEW IMPROVEMENT: SMART DRONE-TRUCK TRANSFER ==========
void LocalSearch::smartDroneTruckTransfer(Solution& solution) {
    //  VALIDATE: Check solution structure
    if (solution.droneRoutes.empty() || solution.truckRoutes.empty()) {
        return;  // Nothing to transfer
    }
    
    struct Transfer {
        int customer;
        size_t fromDrone;
        size_t fromTrip;
        size_t toTruck;
        size_t bestPos;
        double saving;
    };
    
    std::vector<Transfer> transfers;
    
    // Tìm các drone trips có thể chuyển sang truck
    for (size_t d = 0; d < solution.droneRoutes.size(); d++) {
        auto& trips = solution.droneRoutes[d];
        
        for (size_t t = 0; t < trips.size(); t++) {
            if (trips[t].customers.empty()) continue;
            
            // Chỉ xét trips có 1 customer (dễ chuyển)
            if (trips[t].customers.size() == 1) {
                int cust = trips[t].customers[0];
                
                //  VALIDATE: Check if customer ID is valid
                if (cust < 1 || cust > (int)instance.customers.size()) {
                    continue;  // Skip invalid customer IDs
                }
                
                double droneDist = 2 * instance.getDistance(0, cust);
                
                // Tìm truck và vị trí tốt nhất
                int bestTruck = -1;
                int bestPos = -1;
                double minInsertion = INF;
                
                for (size_t tr = 0; tr < solution.truckRoutes.size(); tr++) {
                    auto& truck = solution.truckRoutes[tr].customers;
                    
                    // Thử insert vào mọi vị trí
                    for (size_t pos = 0; pos <= truck.size(); pos++) {
                        int prev = (pos == 0) ? 0 : truck[pos - 1];
                        int next = (pos == truck.size()) ? 0 : truck[pos];
                        
                        double insertCost = instance.getDistance(prev, cust) 
                                          + instance.getDistance(cust, next)
                                          - instance.getDistance(prev, next);
                        
                        if (insertCost < minInsertion) {
                            minInsertion = insertCost;
                            bestTruck = tr;
                            bestPos = pos;
                        }
                    }
                }
                
                // Nếu truck insert rẻ hơn drone trip (85% threshold)
                if (minInsertion < droneDist * 0.85) {
                    Transfer tf;
                    tf.customer = cust;
                    tf.fromDrone = d;
                    tf.fromTrip = t;
                    tf.toTruck = bestTruck;
                    tf.bestPos = bestPos;
                    tf.saving = droneDist - minInsertion;
                    transfers.push_back(tf);
                }
            }
        }
    }
    
    // Sort by saving và apply transfers
    std::sort(transfers.begin(), transfers.end(), 
              [](const Transfer& a, const Transfer& b) { 
                  return a.saving > b.saving; 
              });
    
    // Apply top transfers (limit để tránh over-transfer)
    std::set<std::pair<size_t, size_t>> processed;  // Track processed (drone, trip) pairs
    int applied = 0;
    
    for (const auto& tf : transfers) {
        if (applied >= 3) break;  // Limit số lượng transfers per iteration
        
        auto key = std::make_pair(tf.fromDrone, tf.fromTrip);
        if (processed.find(key) != processed.end()) continue;
        
        // Verify trip still exists at this index (may have shifted)
        if (tf.fromTrip >= solution.droneRoutes[tf.fromDrone].size()) continue;
        if (solution.droneRoutes[tf.fromDrone][tf.fromTrip].customers.empty()) continue;
        if (solution.droneRoutes[tf.fromDrone][tf.fromTrip].customers[0] != tf.customer) continue;
        
        // Apply transfer
        solution.droneRoutes[tf.fromDrone].erase(
            solution.droneRoutes[tf.fromDrone].begin() + tf.fromTrip
        );
        
        solution.truckRoutes[tf.toTruck].customers.insert(
            solution.truckRoutes[tf.toTruck].customers.begin() + tf.bestPos,
            tf.customer
        );
        
        processed.insert(key);
        applied++;
    }
}

// ========== NEW IMPROVEMENT: OPTIMIZE DRONE TRIP SEQUENCES ==========
void LocalSearch::optimizeDroneTrips(Solution& solution) {
    for (auto& trips : solution.droneRoutes) {
        for (auto& trip : trips) {
            if (trip.customers.size() <= 2) continue;
            
            // 2-opt cho drone trip
            bool improved = true;
            int maxIter = 10;
            int iter = 0;
            
            while (improved && iter < maxIter) {
                improved = false;
                iter++;
                int n = trip.customers.size();
                
                for (int i = 0; i < n - 1; i++) {
                    for (int j = i + 1; j < n; j++) {
                        int prev_i = (i == 0) ? 0 : trip.customers[i - 1];
                        int curr_i = trip.customers[i];
                        int curr_j = trip.customers[j];
                        int next_j = (j == n - 1) ? 0 : trip.customers[j + 1];
                        
                        double oldDist = instance.getDistance(prev_i, curr_i) 
                                       + instance.getDistance(curr_j, next_j);
                        double newDist = instance.getDistance(prev_i, curr_j) 
                                       + instance.getDistance(curr_i, next_j);
                        
                        if (newDist < oldDist - 0.01) {
                            std::reverse(trip.customers.begin() + i, 
                                       trip.customers.begin() + j + 1);
                            improved = true;
                            break;
                        }
                    }
                    if (improved) break;
                }
            }
        }
    }
}


// ========== GENI OPERATOR: Generalized Insertion ==========
// Remove a customer and reinsert at best position in entire solution
void LocalSearch::geniOperator(Solution& solution, int maxIterations) {
    bool improved = true;
    int iteration = 0;
    
    while (improved && iteration < maxIterations) {
        improved = false;
        iteration++;
        
        // Collect all customers from all routes
        std::vector<int> allCustomers;
        
        // From truck routes
        for (const auto& route : solution.truckRoutes) {
            for (int custId : route.customers) {
                allCustomers.push_back(custId);
            }
        }
        
        // From drone routes
        for (const auto& trips : solution.droneRoutes) {
            for (const auto& trip : trips) {
                for (int custId : trip.customers) {
                    allCustomers.push_back(custId);
                }
            }
        }
        
        if (allCustomers.empty()) break;
        
        // Try to relocate each customer
        for (int custId : allCustomers) {
            // ✅ VALIDATE: custId must be valid (1 to numCustomers)
            if (custId < 1 || custId > (int)instance.customers.size()) {
                continue;  // Skip invalid customer IDs
            }
            
            Solution current = solution;
            
            // ========== STEP 1: Remove customer ==========
            bool removed = false;
            
            // Try to remove from truck routes
            for (auto& route : current.truckRoutes) {
                auto it = std::find(route.customers.begin(), 
                                   route.customers.end(), custId);
                if (it != route.customers.end()) {
                    route.customers.erase(it);
                    removed = true;
                    break;
                }
            }
            
            // If not in truck, remove from drone routes
            if (!removed) {
                for (auto& trips : current.droneRoutes) {
                    for (auto& trip : trips) {
                        auto it = std::find(trip.customers.begin(), 
                                           trip.customers.end(), custId);
                        if (it != trip.customers.end()) {
                            trip.customers.erase(it);
                            removed = true;
                            break;
                        }
                    }
                    if (removed) break;
                }
            }
            
            if (!removed) continue;
            
            // ========== STEP 2: Find best insertion position ==========
            double bestCost = INF;
            Solution bestSolution = current;
            int custIdx = custId - 1;
            
            // A. Try inserting into TRUCK routes
            for (int truckId = 0; truckId < (int)current.truckRoutes.size(); truckId++) {
                auto& route = current.truckRoutes[truckId];
                
                // Trucks have unlimited capacity in this problem, no need to check
                
                // Try each insertion position
                for (int pos = 0; pos <= (int)route.customers.size(); pos++) {
                    Solution test = current;
                    test.truckRoutes[truckId].customers.insert(
                        test.truckRoutes[truckId].customers.begin() + pos, custId);
                    
                    evaluator.evaluate(test);
                    
                    double testCost = test.systemCompletionTime + test.totalSampleWaitingTime;
                    if (testCost < bestCost) {
                        bestCost = testCost;
                        bestSolution = test;
                    }
                }
            }
            
            // B. Try inserting into DRONE routes
            if (!instance.customers[custIdx].isStaffOnly) {
                for (int droneId = 0; droneId < (int)current.droneRoutes.size(); droneId++) {
                    auto& trips = current.droneRoutes[droneId];
                    
                    // Try existing trips
                    for (int tripIdx = 0; tripIdx < (int)trips.size(); tripIdx++) {
                        auto& trip = trips[tripIdx];
                        
                        // Check capacity
                        double totalDemand = 0;
                        for (int c : trip.customers) {
                            totalDemand += instance.customers[c - 1].demand;
                        }
                        totalDemand += instance.customers[custIdx].demand;
                        
                        if (totalDemand > instance.droneParams[droneId].maxCapacity) {
                            continue;
                        }
                        
                        // Try each position in this trip
                        for (int pos = 0; pos <= (int)trip.customers.size(); pos++) {
                            Solution test = current;
                            test.droneRoutes[droneId][tripIdx].customers.insert(
                                test.droneRoutes[droneId][tripIdx].customers.begin() + pos, 
                                custId);
                            
                            evaluator.evaluate(test);
                            
                            double testCost = test.systemCompletionTime + test.totalSampleWaitingTime;
                            if (testCost < bestCost) {
                                bestCost = testCost;
                                bestSolution = test;
                            }
                        }
                    }
                    
                    // Try creating new trip with just this customer
                    Solution test = current;
                    Route newTrip;
                    newTrip.customers.push_back(custId);
                    test.droneRoutes[droneId].push_back(newTrip);
                    
                    evaluator.evaluate(test);
                    
                    double testCost = test.systemCompletionTime + test.totalSampleWaitingTime;
                    if (testCost < bestCost) {
                        bestCost = testCost;
                        bestSolution = test;
                    }
                }
            }
            
            // ========== STEP 3: Apply best move if improvement found ==========
            evaluator.evaluate(solution);
            double currentCost = solution.systemCompletionTime + solution.totalSampleWaitingTime;
            if (bestCost < currentCost - 0.01) {
                solution = bestSolution;
                improved = true;
                break;  // Restart with new solution
            }
        }
    }
    
    // Final cleanup: remove empty routes/trips
    for (auto& route : solution.truckRoutes) {
        // Keep route structure but customers might be empty
    }
    
    for (auto& trips : solution.droneRoutes) {
        trips.erase(std::remove_if(trips.begin(), trips.end(),
                                   [](const Route& trip) { return trip.customers.empty(); }),
                   trips.end());
    }
    
    evaluator.evaluate(solution);
}
