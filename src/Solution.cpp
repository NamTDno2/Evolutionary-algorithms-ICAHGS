#include "Solution.h"
#include <algorithm>
#include <cmath>
#include <vector> // Thêm thư viện này

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
    double returnTime = currentTime;
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

double SolutionEvaluator::calculateTruckTravelTime(double startTime, double distance) {
    double time = 0;
    double remainingDist = distance;
    double currentTime = startTime;
    
    while (remainingDist > 1e-6) {
        double sigma = getSpeedFactor(currentTime);
        double speed = instance.truckParams.maxSpeed * sigma;
        
        double intervalEnd = INF;
        for (const auto& interval : instance.truckParams.timeIntervals) {
            if (currentTime >= interval.startTime && currentTime < interval.endTime) {
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
        
        currentLoad -= instance.customers[custId - 1].demand;
        prevNode = custId;
    }
    
    double distance = instance.getDistance(prevNode, 0);
    double travelTime = distance / instance.droneParams.cruiseSpeed;
    double power = instance.droneParams.beta * currentLoad + instance.droneParams.gamma;
    totalEnergy += power * travelTime;
    
    return totalEnergy / 1000.0;
}

double SolutionEvaluator::getSpeedFactor(double time) const {
    for (const auto& interval : instance.truckParams.timeIntervals) {
        if (time >= interval.startTime && time < interval.endTime) {
            return interval.sigma;
        }
    }
    if (!instance.truckParams.timeIntervals.empty()) {
        return instance.truckParams.timeIntervals.back().sigma;
    }
    return 1.0;
}


// === ParetoRanking Class ===

void ParetoRanking::nonDominatedSorting(vector<Solution*>& solutions) {
    int n = solutions.size();
    vector<int> dominationCount(n, 0);
    vector<vector<int>> dominatedSolutions(n);
    vector<vector<int>> fronts;
    
    vector<int> currentFront;
    
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
    
    int rank = 1;
    while (!currentFront.empty()) {
        vector<int> nextFront;
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