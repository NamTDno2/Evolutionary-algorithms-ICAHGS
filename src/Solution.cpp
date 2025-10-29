#include "Solution.h"
#include <algorithm>
#include <cmath>

void SolutionEvaluator::evaluate(Solution& solution) {
    double maxCompletionTime = 0;
    double totalWaiting = 0;
    
    // Evaluate truck routes
    for (size_t i = 0; i < solution.truckRoutes.size(); i++) {
        evaluateTruckRoute(solution.truckRoutes[i], i);
        maxCompletionTime = std::max(maxCompletionTime, 
                                     solution.truckRoutes[i].completionTime);
        totalWaiting += solution.truckRoutes[i].totalWaitingTime;
    }
    
    // Evaluate drone routes
    for (size_t i = 0; i < solution.droneRoutes.size(); i++) {
        for (auto& route : solution.droneRoutes[i]) {
            if (evaluateDroneRoute(route, i)) {
                maxCompletionTime = std::max(maxCompletionTime, 
                                           route.completionTime);
                totalWaiting += route.totalWaitingTime;
            } else {
                // Infeasible route (energy constraint violated)
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
    int prevNode = 0;  // Start from depot
    
    for (int custId : route.customers) {
        // Travel time
        double distance = instance.getDistance(prevNode, custId);
        double travelTime = calculateTruckTravelTime(currentTime, distance);
        currentTime += travelTime;
        
        // Service time
        double serviceTime = instance.customers[custId - 1].serviceTimeTruck;
        double collectTime = currentTime;
        currentTime += serviceTime;
        
        prevNode = custId;
    }
    
    // Return to depot
    double distance = instance.getDistance(prevNode, 0);
    double travelTime = calculateTruckTravelTime(currentTime, distance);
    currentTime += travelTime;
    
    route.completionTime = currentTime;
    
    // Calculate waiting time for all samples
    double returnTime = currentTime;
    currentTime = 0;
    prevNode = 0;
    
    for (int custId : route.customers) {
        double distance = instance.getDistance(prevNode, custId);
        double travelTime = calculateTruckTravelTime(currentTime, distance);
        currentTime += travelTime;
        
        double serviceTime = instance.customers[custId - 1].serviceTimeTruck;
        double collectTime = currentTime;
        currentTime += serviceTime;
        
        // Waiting time = return time - collection time
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
    
    // Check energy feasibility
    double energyConsumed = calculateDroneEnergy(route);
    if (energyConsumed > instance.droneParams.maxEnergy) {
        return false;  // Infeasible
    }
    
    // Check capacity
    double totalLoad = 0;
    for (int custId : route.customers) {
        totalLoad += instance.customers[custId - 1].demand;
    }
    if (totalLoad > instance.droneParams.maxCapacity) {
        return false;  // Infeasible
    }
    
    // Calculate completion time
    double currentTime = 0;
    double currentLoad = totalLoad;
    int prevNode = 0;
    
    for (int custId : route.customers) {
        double distance = instance.getDistance(prevNode, custId);
        double travelTime = distance / instance.droneParams.cruiseSpeed;
        currentTime += travelTime;
        
        double serviceTime = instance.customers[custId - 1].serviceTimeDrone;
        currentTime += serviceTime;
        
        currentLoad -= instance.customers[custId - 1].demand;
        prevNode = custId;
    }
    
    // Return to depot
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

double SolutionEvaluator::calculateTruckTravelTime(double startTime, 
                                                    double distance) {
    double time = 0;
    double remainingDist = distance;
    double currentTime = startTime;
    
    while (remainingDist > 1e-6) {
        double sigma = getSpeedFactor(currentTime);
        double speed = instance.truckParams.maxSpeed * sigma;
        
        // Find time interval boundary
        double intervalEnd = INF;
        for (const auto& interval : instance.truckParams.timeIntervals) {
            if (currentTime >= interval.startTime && 
                currentTime < interval.endTime) {
                intervalEnd = interval.endTime;
                break;
            }
        }
        
        double timeToIntervalEnd = intervalEnd - currentTime;
        double distInInterval = speed * timeToIntervalEnd;
        
        if (distInInterval >= remainingDist) {
            time += remainingDist / speed;
            break;
        } else {
            time += timeToIntervalEnd;
            remainingDist -= distInInterval;
            currentTime = intervalEnd;
        }
    }
    
    return time;
}

double SolutionEvaluator::calculateDroneEnergy(const Route& route) {
    if (route.isEmpty()) return 0;
    
    double totalEnergy = 0;
    double currentLoad = 0;
    
    // Calculate total load
    for (int custId : route.customers) {
        currentLoad += instance.customers[custId - 1].demand;
    }
    
    int prevNode = 0;
    
    for (int custId : route.customers) {
        double distance = instance.getDistance(prevNode, custId);
        double travelTime = distance / instance.droneParams.cruiseSpeed;
        
        // Power = beta * load + gamma (Watts)
        double power = instance.droneParams.beta * currentLoad + 
                       instance.droneParams.gamma;
        
        // Energy = Power * Time (Joules = Watt * seconds)
        double energy = power * travelTime;
        totalEnergy += energy;
        
        // Update load after serving customer
        currentLoad -= instance.customers[custId - 1].demand;
        prevNode = custId;
    }
    
    // Return to depot
    double distance = instance.getDistance(prevNode, 0);
    double travelTime = distance / instance.droneParams.cruiseSpeed;
    double power = instance.droneParams.beta * currentLoad + 
                   instance.droneParams.gamma;
    totalEnergy += power * travelTime;
    
    // Convert to kJ
    return totalEnergy / 1000.0;
}

double SolutionEvaluator::getSpeedFactor(double time) const {
    for (const auto& interval : instance.truckParams.timeIntervals) {
        if (time >= interval.startTime && time < interval.endTime) {
            return interval.sigma;
        }
    }
    // Return last interval's sigma if beyond all intervals
    if (!instance.truckParams.timeIntervals.empty()) {
        return instance.truckParams.timeIntervals.back().sigma;
    }
    return 1.0;
}

void ParetoRanking::nonDominatedSorting(std::vector<Solution*>& solutions) {
    int n = solutions.size();
    std::vector<int> dominationCount(n, 0);
    std::vector<std::vector<int>> dominatedSolutions(n);
    std::vector<std::vector<int>> fronts;
    std::vector<int> currentFront;
    
    // Find domination relationships
    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
            if (solutions[i]->dominates(*solutions[j])) {
                dominatedSolutions[i].push_back(j);
                dominationCount[j]++;
            } else if (solutions[j]->dominates(*solutions[i])) {
                dominatedSolutions[j].push_back(i);
                dominationCount[i]++;
            }
        }
        
        if (dominationCount[i] == 0) {
            currentFront.push_back(i);
            solutions[i]->paretoRank = 1;
        }
    }
    
    fronts.push_back(currentFront);
    
    // Build subsequent fronts
    int rank = 1;
    while (!currentFront.empty()) {
        std::vector<int> nextFront;
        for (int i : currentFront) {
            for (int j : dominatedSolutions[i]) {
                dominationCount[j]--;
                if (dominationCount[j] == 0) {
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
    
    // Calculate crowding distance for each front
    for (auto& front : fronts) {
        std::vector<Solution*> frontSols;
        for (int idx : front) {
            frontSols.push_back(solutions[idx]);
        }
        calculateCrowdingDistance(frontSols);
    }
}

void ParetoRanking::calculateCrowdingDistance(std::vector<Solution*>& front) {
    int n = front.size();
    if (n <= 2) {
        for (auto* sol : front) {
            sol->crowdingDistance = INF;
        }
        return;
    }
    
    // Initialize
    for (auto* sol : front) {
        sol->crowdingDistance = 0;
    }
    
    // For each objective
    for (int obj = 0; obj < 2; obj++) {
        // Sort by objective
        std::sort(front.begin(), front.end(), [obj](Solution* a, Solution* b) {
            if (obj == 0) return a->systemCompletionTime < b->systemCompletionTime;
            else return a->totalSampleWaitingTime < b->totalSampleWaitingTime;
        });
        
        // Boundary points have infinite distance
        front->crowdingDistance = INF;
        front[n-1]->crowdingDistance = INF;
        
        // Normalized range
        double objMin = (obj == 0) ? front->systemCompletionTime 
                                   : front->totalSampleWaitingTime;
        double objMax = (obj == 0) ? front[n-1]->systemCompletionTime 
                                   : front[n-1]->totalSampleWaitingTime;
        double range = objMax - objMin;
        
        if (range < 1e-6) continue;
        
        // Calculate crowding distance
        for (int i = 1; i < n - 1; i++) {
            double prev = (obj == 0) ? front[i-1]->systemCompletionTime 
                                     : front[i-1]->totalSampleWaitingTime;
            double next = (obj == 0) ? front[i+1]->systemCompletionTime 
                                     : front[i+1]->totalSampleWaitingTime;
            front[i]->crowdingDistance += (next - prev) / range;
        }
    }
}
