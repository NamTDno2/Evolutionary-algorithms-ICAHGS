#include "LocalSearch.h"
#include <algorithm>
#include <queue>
#include <iostream> // Thêm để debug

Solution LocalSearch::improve(const Solution& solution, int maxIterations) {
    Solution current = solution;
    Solution best = solution;
    
    tabuList.clear();
    int iterWithoutImprovement = 0;
    
    for (int iter = 0; iter < maxIterations; iter++) {
        Move bestMove = findBestMove(current);
        
        if (bestMove.customer1 == -1) {
            // No feasible move found
            break;
        }
        
        // Apply move
        Solution neighbor = applyMove(current, bestMove);
        evaluator.evaluate(neighbor);
        
        // Update tabu list
        updateTabuList(bestMove.customer1, static_cast<int>(bestMove.type));
        if (bestMove.customer2 != -1) {
            updateTabuList(bestMove.customer2, static_cast<int>(bestMove.type));
        }
        
        // Check if improved
        if (neighbor.dominates(best)) {
            best = neighbor;
            iterWithoutImprovement = 0;
        } else {
            iterWithoutImprovement++;
        }
        
        current = neighbor;
        
        // Early stopping
        if (iterWithoutImprovement > 20) {
            break;
        }
    }
    
    return best;
}

LocalSearch::Move LocalSearch::findBestMove(const Solution& solution) {
    Move bestMove;
    bestMove.deltaCost = INF;
    
    // Collect all customers in solution
    std::vector<int> allCustomers;
    
    // From truck routes
    for (const auto& route : solution.truckRoutes) {
        for (int cust : route.customers) {
            allCustomers.push_back(cust);
        }
    }
    
    // From drone routes
    for (const auto& trips : solution.droneRoutes) {
        for (const auto& trip : trips) {
            for (int cust : trip.customers) {
                allCustomers.push_back(cust);
            }
        }
    }
    
    // Try RELOCATE moves
    for (int cust : allCustomers) {
        if (isTabu(cust, Move::RELOCATE)) continue;
        
        // Try moving to different positions in truck routes
        for (size_t truckId = 0; truckId < solution.truckRoutes.size(); truckId++) {
            const auto& route = solution.truckRoutes[truckId];
            
            for (size_t pos = 0; pos <= route.customers.size(); pos++) {
                // Create neighbor
                Move move;
                move.type = Move::RELOCATE;
                move.customer1 = cust;
                move.toRoute = truckId;
                move.toPos = pos;
                
                Solution neighbor = applyMove(solution, move);
                evaluator.evaluate(neighbor);
                
                double delta = calculateDelta(solution, neighbor);
                
                if (delta < bestMove.deltaCost && neighbor.systemCompletionTime < INF) {
                    bestMove = move;
                    bestMove.deltaCost = delta;
                }
            }
        }
        
        // Try moving to drone routes (if flexible customer)
        const Customer& customer = instance.customers[cust - 1];
        if (!customer.isStaffOnly) {
            for (size_t droneId = 0; droneId < solution.droneRoutes.size(); droneId++) {
                // Try adding to new trip
                Move move;
                move.type = Move::RELOCATE;
                move.customer1 = cust;
                move.toRoute = droneId + 1000;  // Offset to distinguish from truck
                
                Solution neighbor = applyMove(solution, move);
                evaluator.evaluate(neighbor);
                
                double delta = calculateDelta(solution, neighbor);
                
                if (delta < bestMove.deltaCost && neighbor.systemCompletionTime < INF) {
                    bestMove = move;
                    bestMove.deltaCost = delta;
                }
            }
        }
    }
    
    // Try SWAP moves (simplified version)
    for (size_t i = 0; i < allCustomers.size(); i++) {
        for (size_t j = i + 1; j < allCustomers.size(); j++) {
            int cust1 = allCustomers[i];
            int cust2 = allCustomers[j];
            
            if (isTabu(cust1, Move::SWAP) || isTabu(cust2, Move::SWAP)) continue;
            
            Move move;
            move.type = Move::SWAP;
            move.customer1 = cust1;
            move.customer2 = cust2;
            
            Solution neighbor = applyMove(solution, move);
            evaluator.evaluate(neighbor);
            
            double delta = calculateDelta(solution, neighbor);
            
            if (delta < bestMove.deltaCost && neighbor.systemCompletionTime < INF) {
                bestMove = move;
                bestMove.deltaCost = delta;
            }
        }
    }
    
    return bestMove;
}

Solution LocalSearch::applyMove(const Solution& solution, const Move& move) {
    Solution result = solution;
    
    if (move.type == Move::RELOCATE) {
        // Remove customer from current position
        bool found = false;
        
        // Try truck routes
        for (auto& route : result.truckRoutes) {
            auto it = std::find(route.customers.begin(), route.customers.end(), 
                              move.customer1);
            if (it != route.customers.end()) {
                route.customers.erase(it);
                found = true;
                break;
            }
        }
        
        // Try drone routes
        if (!found) {
            for (auto& trips : result.droneRoutes) {
                for (auto& trip : trips) {
                    auto it = std::find(trip.customers.begin(), trip.customers.end(),
                                      move.customer1);
                    if (it != trip.customers.end()) {
                        trip.customers.erase(it);
                        found = true;
                        break; // Dừng ngay khi tìm thấy và xóa
                    }
                }
                if (found) break; // Dừng vòng lặp ngoài nếu đã tìm thấy
            }
        }
        
        // Insert at new position
        if (move.toRoute < 1000) {
            // Insert into truck route
            auto& target_customers = result.truckRoutes[move.toRoute].customers;
            size_t insert_pos = move.toPos;
            if (insert_pos > target_customers.size()) {
                insert_pos = target_customers.size();
            }
            target_customers.insert(
                target_customers.begin() + insert_pos,
                move.customer1);
        } else {
            // Insert into drone route (new trip)
            int droneId = move.toRoute - 1000;
            Route newTrip;
            newTrip.customers.push_back(move.customer1);
            result.droneRoutes[droneId].push_back(newTrip);
        }
        
    } else if (move.type == Move::SWAP) {
        // --- FIX: Sử dụng index thay vì con trỏ để tránh lỗi bộ nhớ ---
        int route_type1 = -1, route_idx1 = -1, trip_idx1 = -1, cust_idx1 = -1;
        int route_type2 = -1, route_idx2 = -1, trip_idx2 = -1, cust_idx2 = -1;

        // Tìm vị trí của customer1
        for (size_t i = 0; i < result.truckRoutes.size() && route_type1 == -1; ++i) {
            for (size_t j = 0; j < result.truckRoutes[i].customers.size(); ++j) {
                if (result.truckRoutes[i].customers[j] == move.customer1) {
                    route_type1 = 0; route_idx1 = i; cust_idx1 = j; break;
                }
            }
        }
        if (route_type1 == -1) {
            for (size_t i = 0; i < result.droneRoutes.size() && route_type1 == -1; ++i) {
                for (size_t j = 0; j < result.droneRoutes[i].size(); ++j) {
                    for (size_t k = 0; k < result.droneRoutes[i][j].customers.size(); ++k) {
                        if (result.droneRoutes[i][j].customers[k] == move.customer1) {
                            route_type1 = 1; route_idx1 = i; trip_idx1 = j; cust_idx1 = k; break;
                        }
                    }
                    if (route_type1 != -1) break;
                }
            }
        }

        // Tìm vị trí của customer2
        for (size_t i = 0; i < result.truckRoutes.size() && route_type2 == -1; ++i) {
            for (size_t j = 0; j < result.truckRoutes[i].customers.size(); ++j) {
                if (result.truckRoutes[i].customers[j] == move.customer2) {
                    route_type2 = 0; route_idx2 = i; cust_idx2 = j; break;
                }
            }
        }
        if (route_type2 == -1) {
            for (size_t i = 0; i < result.droneRoutes.size() && route_type2 == -1; ++i) {
                for (size_t j = 0; j < result.droneRoutes[i].size(); ++j) {
                    for (size_t k = 0; k < result.droneRoutes[i][j].customers.size(); ++k) {
                        if (result.droneRoutes[i][j].customers[k] == move.customer2) {
                            route_type2 = 1; route_idx2 = i; trip_idx2 = j; cust_idx2 = k; break;
                        }
                    }
                    if (route_type2 != -1) break;
                }
            }
        }

        // Thực hiện hoán đổi nếu tìm thấy cả hai
        if (route_type1 != -1 && route_type2 != -1) {
            int& cust1_ref = (route_type1 == 0) ? result.truckRoutes[route_idx1].customers[cust_idx1] : result.droneRoutes[route_idx1][trip_idx1].customers[cust_idx1];
            int& cust2_ref = (route_type2 == 0) ? result.truckRoutes[route_idx2].customers[cust_idx2] : result.droneRoutes[route_idx2][trip_idx2].customers[cust_idx2];
            std::swap(cust1_ref, cust2_ref);
        }
    }
    
    return result;
}

bool LocalSearch::isTabu(int customer, int moveType) const {
    return tabuList.find(std::make_pair(customer, moveType)) != tabuList.end();
}

void LocalSearch::updateTabuList(int customer, int moveType) {
    tabuList.insert({customer, moveType});
    
    // Keep tabu list size limited
    if (tabuList.size() > static_cast<size_t>(tabuTenure)) {
        tabuList.erase(tabuList.begin());
    }
}

double LocalSearch::calculateDelta(const Solution& current, 
                                   const Solution& neighbor) {
    // Weighted sum of objectives
    double w1 = 0.5, w2 = 0.5;
    
    double delta1 = neighbor.systemCompletionTime - current.systemCompletionTime;
    double delta2 = neighbor.totalSampleWaitingTime - current.totalSampleWaitingTime;
    
    return w1 * delta1 + w2 * delta2;
}
