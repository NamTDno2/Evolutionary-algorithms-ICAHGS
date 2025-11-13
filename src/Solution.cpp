#include "Solution.h"
#include <algorithm>
#include <cmath>
#include <vector> // Thêm thư viện này
#include <random>      // ← THÊM (cho mt19937_64)
#include <tuple>       // ← THÊM (cho make_tuple)
#include <map>         // ← THÊM (cho map)
#include <cstdint>

using namespace std;

// === SolutionEvaluator Class ===

void SolutionEvaluator::evaluate(Solution& solution) {
    double maxCompletionTime = 0;
    double totalWaiting = 0;
    
    // Evaluate truck routes
    for (size_t i = 0; i < solution.truckRoutes.size(); i++) {
        evaluateTruckRoute(solution.truckRoutes[i], i);
        maxCompletionTime = max(maxCompletionTime, 
                                  solution.truckRoutes[i].completionTime);
        totalWaiting += solution.truckRoutes[i].totalWaitingTime;
    }
    
    // Evaluate drone routes
    for (size_t i = 0; i < solution.droneRoutes.size(); i++) {
        for (auto& route : solution.droneRoutes[i]) {
            if (evaluateDroneRoute(route, i)) {
                maxCompletionTime = max(maxCompletionTime, 
                                          route.completionTime);
                totalWaiting += route.totalWaitingTime;
            } else {
                // Infeasible route
                solution.systemCompletionTime = INF;
                solution.totalSampleWaitingTime = INF;
                return;
            }
        }
    }
    
    solution.systemCompletionTime = maxCompletionTime;
    solution.totalSampleWaitingTime = totalWaiting;
}

void SolutionEvaluator::evaluateTruckRoute(Route& route, int truckId) {
    if (route.isEmpty()) {
        route.completionTime = 0;
        route.totalWaitingTime = 0;
        return;
    }
    
    double currentTime = 0;
    double totalWaiting = 0;
    int prevNode = 0; // Start from depot
    
    for (int custId : route.customers) {
        double distance = instance.getDistance(prevNode, custId);
        double travelTime = calculateTruckTravelTime(currentTime, distance);
        currentTime += travelTime;
        
        double serviceTime = instance.customers[custId - 1].serviceTimeTruck;
        currentTime += serviceTime;
        
        prevNode = custId;
    }
    
    // Return to depot
    double distance = instance.getDistance(prevNode, 0);
    double travelTime = calculateTruckTravelTime(currentTime, distance);
    currentTime += travelTime;
    
    route.completionTime = currentTime;
    
    // Calculate waiting time
    double returnTime = currentTime;    // Lưu thời gian quay về depot
    currentTime = 0;
    prevNode = 0;
    
    for (int custId : route.customers) {
        double distance = instance.getDistance(prevNode, custId);
        double travelTime = calculateTruckTravelTime(currentTime, distance);
        currentTime += travelTime;
        
        double collectTime = currentTime;
        double serviceTime = instance.customers[custId - 1].serviceTimeTruck;
        currentTime += serviceTime;
        
        totalWaiting += (returnTime - collectTime);
        
        prevNode = custId;
    }
    
    route.totalWaitingTime = totalWaiting;
}

bool SolutionEvaluator::evaluateDroneRoute(Route& route, int droneId) {
    if (route.isEmpty()) {
        route.completionTime = 0;
        route.totalWaitingTime = 0;
        return true;
    }
    
    double totalLoad = 0;
    for (int custId : route.customers) {
        totalLoad += instance.customers[custId - 1].demand;
    }
    if (totalLoad > instance.droneParams.maxCapacity) {
        return false;
    }

    if (calculateDroneEnergy(route) > instance.droneParams.maxEnergy) {
        return false;
    }

    double currentTime = 0;
    int prevNode = 0;
    
    for (int custId : route.customers) {
        double distance = instance.getDistance(prevNode, custId);
        double travelTime = distance / instance.droneParams.cruiseSpeed;
        currentTime += travelTime;
        
        double serviceTime = instance.customers[custId - 1].serviceTimeDrone;
        currentTime += serviceTime;
        
        prevNode = custId;
    }
    
    double distance = instance.getDistance(prevNode, 0);
    currentTime += distance / instance.droneParams.cruiseSpeed;
    
    route.completionTime = currentTime;
    
    // Calculate waiting time
    double returnTime = currentTime;
    currentTime = 0;
    prevNode = 0;
    
    double totalWaiting = 0;
    for (int custId : route.customers) {
        double distance = instance.getDistance(prevNode, custId);
        double travelTime = distance / instance.droneParams.cruiseSpeed;
        currentTime += travelTime;
        
        double collectTime = currentTime;
        double serviceTime = instance.customers[custId - 1].serviceTimeDrone;
        currentTime += serviceTime;
        
        totalWaiting += (returnTime - collectTime);
        prevNode = custId;
    }
    
    route.totalWaitingTime = totalWaiting;
    return true;
}

// TÍNH THỜI GIAN DI CHUYỂN CÓ PHỤ THUỘC VÀO THỜI ĐIỂM
double SolutionEvaluator::calculateTruckTravelTime(double startTime, double distance) {
    double time = 0;
    double remainingDist = distance;
    double currentTime = startTime;
    
    while (remainingDist > 1e-6) {  // Khi còn quãng đường
        double sigma = getSpeedFactor(currentTime);
        double speed = instance.truckParams.maxSpeed * sigma;
        
        double intervalEnd = INF;
        for (const auto& interval : instance.truckParams.timeIntervals) {
            if (currentTime >= interval.startTime && currentTime < interval.endTime) {
                intervalEnd = interval.endTime;
                break;
            }
        }
        
        // Tính quãng đường có thể đi trong interval này
        double timeToIntervalEnd = intervalEnd - currentTime;
        double distInInterval = speed * timeToIntervalEnd;
        
        // Nếu quãng đường còn lại có thể đi hết trong interval → xong
        if (distInInterval >= remainingDist) {
            time += remainingDist / speed;
            break;
        } else {
            // Nếu chưa hết → di chuyển hết interval này, chuyển sang interval tiếp
            time += timeToIntervalEnd;
            remainingDist -= distInInterval;
            currentTime = intervalEnd;
        }
    }
    
    return time;
}

double SolutionEvaluator::calculateDroneEnergy(const Route& route) {
    
    // TÍNH NĂNG LƯỢNG TIÊU THỤ CỦA DRONE
    // Energy = (β * Load + γ) * flightTime

    if (route.isEmpty()) return 0;
    
    double totalEnergy = 0;
    double currentLoad = 0;
    
    for (int custId : route.customers) {
        currentLoad += instance.customers[custId - 1].demand;
    }
    
    int prevNode = 0;
    
    for (int custId : route.customers) {
        double distance = instance.getDistance(prevNode, custId);
        double travelTime = distance / instance.droneParams.cruiseSpeed;
        
        double power = instance.droneParams.beta * currentLoad + instance.droneParams.gamma;
        double energy = power * travelTime;
        totalEnergy += energy;
        
        // Sau khi lấy mẫu, tải trọng giảm
        currentLoad -= instance.customers[custId - 1].demand;
        prevNode = custId;
    }
    
    // Quay về depot (không có tải)
    double distance = instance.getDistance(prevNode, 0);
    double travelTime = distance / instance.droneParams.cruiseSpeed;
    double power = instance.droneParams.beta * currentLoad + instance.droneParams.gamma;
    totalEnergy += power * travelTime;
    
    return totalEnergy / 1000.0;    // Chuyển sang kJ
}

// TÌM HỆ SỐ TỐCĐỘ TẠI THỜI ĐIỂM time
double SolutionEvaluator::getSpeedFactor(double time) const {
    for (const auto& interval : instance.truckParams.timeIntervals) {
        if (time >= interval.startTime && time < interval.endTime) {
            return interval.sigma;
        }
    }
    // Nếu không tìm thấy (vượt quá cuối ngày) → lấy σ của interval cuối cùng
    if (!instance.truckParams.timeIntervals.empty()) {
        return instance.truckParams.timeIntervals.back().sigma;
    }
    return 1.0;
}


// === ParetoRanking Class ===

void ParetoRanking::nonDominatedSorting(vector<Solution*>& solutions) {
    int n = solutions.size();
    vector<int> dominationCount(n, 0);  // Đếm số solution dominate solution i
    vector<vector<int>> dominatedSolutions(n);  // Danh sách solutions bị i dominate
    vector<vector<int>> fronts; // Lưu các fronts
    
    vector<int> currentFront;
    
    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
            if (solutions[i]->dominates(*solutions[j])) {   // i dominate j
                dominatedSolutions[i].push_back(j);
                dominationCount[j]++;
            } else if (solutions[j]->dominates(*solutions[i])) {    // j dominate i
                dominatedSolutions[j].push_back(i);
                dominationCount[i]++;
            }
        }
        
        // TÌM FRONT 1
        if (dominationCount[i] == 0) {  
            currentFront.push_back(i);
            solutions[i]->paretoRank = 1;
        }
    }
    
    fronts.push_back(currentFront);
    
    //  TÌM FRONT 2, 3, ...
    int rank = 1;
    while (!currentFront.empty()) {
        vector<int> nextFront;
        for (int i : currentFront) {
            for (int j : dominatedSolutions[i]) {
                dominationCount[j]--;
                if (dominationCount[j] == 0) {  //  Nếu j không còn ai dominate
                    nextFront.push_back(j);
                    solutions[j]->paretoRank = rank + 1;
                }
            }
        }
        if (!nextFront.empty()) {
            fronts.push_back(nextFront);
        }
        currentFront = nextFront;
        rank++;
    }
    
    for (auto& frontIndexes : fronts) {
        vector<Solution*> frontSols;
        for (int idx : frontIndexes) {
            frontSols.push_back(solutions[idx]);
        }
        calculateCrowdingDistance(frontSols);
    }
}

void ParetoRanking::calculateCrowdingDistance(vector<Solution*>& front) {
    int n = front.size();
    if (n <= 2) {
        for (auto* sol : front) {
            sol->crowdingDistance = INF;
        }
        return;
    }
    
    for (auto* sol : front) {
        sol->crowdingDistance = 0;
    }
    
    for (int obj = 0; obj < 2; obj++) {
        sort(front.begin(), front.end(), [obj](Solution* a, Solution* b) {
            if (obj == 0) return a->systemCompletionTime < b->systemCompletionTime;
            else return a->totalSampleWaitingTime < b->totalSampleWaitingTime;
        });
        
        front[0]->crowdingDistance = INF;
        front[n-1]->crowdingDistance = INF;
        
        double objMin = (obj == 0) ? front[0]->systemCompletionTime 
                                   : front[0]->totalSampleWaitingTime;
        double objMax = (obj == 0) ? front[n-1]->systemCompletionTime 
                                   : front[n-1]->totalSampleWaitingTime;
        double range = objMax - objMin;
        
        if (range < 1e-6) continue;
        
        for (int i = 1; i < n - 1; i++) {
            double prev = (obj == 0) ? front[i-1]->systemCompletionTime 
                                     : front[i-1]->totalSampleWaitingTime;
            double next = (obj == 0) ? front[i+1]->systemCompletionTime 
                                     : front[i+1]->totalSampleWaitingTime;
            front[i]->crowdingDistance += (next - prev) / range;
        }
    }
}
// === SolutionHasher Implementation ===

SolutionHasher::SolutionHasher(int maxCustomers, int maxTrucks, int maxDrones) {
    initializeZobristTable(maxCustomers, maxTrucks, maxDrones);
}

void SolutionHasher::initializeZobristTable(int maxCustomers, int maxTrucks, int maxDrones) {
    std::mt19937_64 rng(42);  // Seed cố định để reproducible
    std::uniform_int_distribution<uint64_t> dist;
    
    int maxRoutes = maxTrucks + maxDrones * 10;  // Drone có nhiều trips
    int maxPositions = maxCustomers;
    
    // Tạo số ngẫu nhiên cho mỗi (customer, route, position)
    for (int customer = 1; customer <= maxCustomers; customer++) {
        for (int route = 0; route < maxRoutes; route++) {
            for (int position = 0; position < maxPositions; position++) {
                auto key = std::make_tuple(customer, route, position);
                zobristTable[key] = dist(rng);
            }
        }
    }
}

uint64_t SolutionHasher::computeHash(const Solution& solution) const {
    uint64_t hash = 0;
    
    // Hash truck routes
    for (size_t truckId = 0; truckId < solution.truckRoutes.size(); truckId++) {
        const Route& route = solution.truckRoutes[truckId];
        
        for (size_t pos = 0; pos < route.customers.size(); pos++) {
            int customer = route.customers[pos];
            int routeId = truckId;  // Route ID for trucks
            
            auto key = std::make_tuple(customer, routeId, (int)pos);
            
            // XOR với Zobrist value
            if (zobristTable.find(key) != zobristTable.end()) {
                hash ^= zobristTable.at(key);
            }
        }
    }
    
    // Hash drone routes
    for (size_t droneId = 0; droneId < solution.droneRoutes.size(); droneId++) {
        const auto& trips = solution.droneRoutes[droneId];
        
        for (size_t tripId = 0; tripId < trips.size(); tripId++) {
            const Route& trip = trips[tripId];
            
            // Route ID cho drone: offset sau truck routes
            int routeId = solution.truckRoutes.size() + droneId * 10 + tripId;
            
            for (size_t pos = 0; pos < trip.customers.size(); pos++) {
                int customer = trip.customers[pos];
                
                auto key = std::make_tuple(customer, routeId, (int)pos);
                
                if (zobristTable.find(key) != zobristTable.end()) {
                    hash ^= zobristTable.at(key);
                }
            }
        }
    }
    
    return hash;
}
