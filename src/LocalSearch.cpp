#include "LocalSearch.h"
#include <algorithm>
#include <queue>
#include <iostream> 

Solution LocalSearch::improve(const Solution& solution, int maxIterations) {
    Solution current = solution;
    Solution best = solution;
    
    tabuList.clear();
    int iterWithoutImprovement = 0;
    
    for (int iter = 0; iter < maxIterations; iter++) {
        Move bestMove = findBestMove(current);
        
        if (bestMove.customer1 == -1) {
            break; // Không tìm thấy nước đi hợp lệ nào
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
        
        if (iterWithoutImprovement > 20) {
            break;
        }
    }
    
    return best;
}

LocalSearch::Move LocalSearch::findBestMove(const Solution& solution) {
    Move bestMove;
    bestMove.deltaCost = INF;
    
    // Collect all customers
    std::vector<int> allCustomers;
    for (const auto& route : solution.truckRoutes) {
        for (int cust : route.customers) allCustomers.push_back(cust);
    }
    for (const auto& trips : solution.droneRoutes) {
        for (const auto& trip : trips) {
            for (int cust : trip.customers) allCustomers.push_back(cust);
        }
    }
    
    // --- 1. Try RELOCATE moves ---
    for (int cust : allCustomers) {
        if (isTabu(cust, Move::RELOCATE)) continue;
        
        // A. Try moving to Truck routes
        for (size_t truckId = 0; truckId < solution.truckRoutes.size(); truckId++) {
            const auto& route = solution.truckRoutes[truckId];
            // Thử mọi vị trí chèn
            for (size_t pos = 0; pos <= route.customers.size(); pos++) {
                Move move;
                move.type = Move::RELOCATE;
                move.customer1 = cust;
                move.toRoute = truckId;
                move.toPos = pos;
                
                Solution neighbor = applyMove(solution, move);
                evaluator.evaluate(neighbor); // Check infeasible (INF cost)
                
                // CHỈ CHẤP NHẬN NẾU HỢP LỆ
                if (neighbor.systemCompletionTime < INF) {
                    double delta = calculateDelta(solution, neighbor);
                    if (delta < bestMove.deltaCost) {
                        bestMove = move;
                        bestMove.deltaCost = delta;
                    }
                }
            }
        }
        
        // B. Try moving to Drone routes
        const Customer& customer = instance.customers[cust - 1];
        if (!customer.isStaffOnly) { // Ràng buộc cứng: StaffOnly không đi Drone
            for (size_t droneId = 0; droneId < solution.droneRoutes.size(); droneId++) {
                // Đơn giản hóa: Chỉ thử tạo chuyến mới (New Trip) để tiết kiệm thời gian tính toán
                // Hoặc chèn vào cuối các chuyến hiện có
                
                // Option 1: New Trip
                Move move;
                move.type = Move::RELOCATE;
                move.customer1 = cust;
                move.toRoute = droneId + 1000; // Offset ID cho Drone
                
                Solution neighbor = applyMove(solution, move);
                evaluator.evaluate(neighbor);
                
                if (neighbor.systemCompletionTime < INF) {
                    double delta = calculateDelta(solution, neighbor);
                    if (delta < bestMove.deltaCost) {
                        bestMove = move;
                        bestMove.deltaCost = delta;
                    }
                }
            }
        }
    }
    
    // --- 2. Try SWAP moves ---
    for (size_t i = 0; i < allCustomers.size(); i++) {
        for (size_t j = i + 1; j < allCustomers.size(); j++) {
            int cust1 = allCustomers[i];
            int cust2 = allCustomers[j];
            
            if (isTabu(cust1, Move::SWAP) || isTabu(cust2, Move::SWAP)) continue;
            
            // Check nhanh: Nếu 1 trong 2 là StaffOnly, không được swap với khách đang ở trên Drone
            // Tuy nhiên để chính xác và đơn giản, ta cứ swap rồi check sau
            
            Move move;
            move.type = Move::SWAP;
            move.customer1 = cust1;
            move.customer2 = cust2;
            
            Solution neighbor = applyMove(solution, move);
            
            // CHECK RÀNG BUỘC StaffOnly TRƯỚC KHI EVALUATE
            bool feasible = true;
            for (const auto& trips : neighbor.droneRoutes) {
                for (const auto& trip : trips) {
                    for (int c : trip.customers) {
                        if (instance.customers[c-1].isStaffOnly) {
                            feasible = false; break;
                        }
                    }
                    if (!feasible) break;
                }
                if (!feasible) break;
            }
            
            if (!feasible) continue; // Bỏ qua nước đi vi phạm
            
            evaluator.evaluate(neighbor);
            
            if (neighbor.systemCompletionTime < INF) {
                double delta = calculateDelta(solution, neighbor);
                if (delta < bestMove.deltaCost) {
                    bestMove = move;
                    bestMove.deltaCost = delta;
                }
            }
        }
    }
    
    return bestMove;
}

Solution LocalSearch::applyMove(const Solution& solution, const Move& move) {
    Solution result = solution;
    
    if (move.type == Move::RELOCATE) {
        bool found = false;
        
        // 1. Xóa khỏi vị trí cũ (Truck)
        for (auto& route : result.truckRoutes) {
            auto it = std::find(route.customers.begin(), route.customers.end(), move.customer1);
            if (it != route.customers.end()) {
                route.customers.erase(it);
                found = true; break;
            }
        }
        // 2. Xóa khỏi vị trí cũ (Drone)
        if (!found) {
            for (auto& trips : result.droneRoutes) {
                for (auto& trip : trips) {
                    auto it = std::find(trip.customers.begin(), trip.customers.end(), move.customer1);
                    if (it != trip.customers.end()) {
                        trip.customers.erase(it);
                        found = true; break;
                    }
                }
                if (found) break;
            }
        }
        
        // 3. Chèn vào vị trí mới
        if (move.toRoute < 1000) { // Vào Truck
            auto& target = result.truckRoutes[move.toRoute].customers;
            size_t pos = std::min((size_t)move.toPos, target.size()); // Safety check
            target.insert(target.begin() + pos, move.customer1);
        } else { // Vào Drone (Tạo chuyến mới)
            int droneId = move.toRoute - 1000;
            Route newTrip;
            newTrip.customers.push_back(move.customer1);
            result.droneRoutes[droneId].push_back(newTrip);
        }
        
    } else if (move.type == Move::SWAP) {
        // Tìm địa chỉ tham chiếu của 2 khách hàng
        int* ptr1 = nullptr;
        int* ptr2 = nullptr;
        
        // Helper lambda để tìm con trỏ tới khách hàng
        auto findCustomerPtr = [&](int c) -> int* {
            for (auto& route : result.truckRoutes) {
                for (auto& val : route.customers) if (val == c) return &val;
            }
            for (auto& trips : result.droneRoutes) {
                for (auto& trip : trips) {
                    for (auto& val : trip.customers) if (val == c) return &val;
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
    
    // Reset objectives để Evaluator tính lại từ đầu
    result.systemCompletionTime = 0;
    result.totalSampleWaitingTime = 0;
    
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
