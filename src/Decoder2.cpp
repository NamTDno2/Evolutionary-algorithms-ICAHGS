#include "Decoder.h"
#include "Chromosome.h"
#include <iostream>
#include <vector>
#include <numeric>   // Cho std::iota
#include <algorithm> // Cho std::shuffle, std::find
#include <random>    // Cho mt19937, uniform_int_distribution
#include <map>

Solution Decoder::decode(const Chromosome &chrom, const Instance &inst)
{
    Solution solution;
    SolutionEvaluator evaluator(inst);

    const int num_technicians_ = inst.numTrucks;
    const int num_drones_ = inst.numDrones;

    // Lấy hoán vị từ Chromosome
    std::vector<int> assignment = chrom.getAssignment();
    std::vector<int> permutation = chrom.getPermutation();

    // Chuẩn bị cấu trúc lời giải
    solution.chrom = chrom;
    solution.truckRoutes.resize(instance.numTrucks);
    solution.droneRoutes.resize(instance.numDrones);

    std::map<int, std::vector<int>> routes_by_vehicle;
    for (int customer_id : permutation)
    {
        //  VALIDATE: customer_id must be valid (1 to numCustomers)
        if (customer_id < 1 || customer_id > (int)instance.customers.size()) {
            std::cerr << "WARNING: Invalid customer_id " << customer_id 
                      << " in permutation (valid range: 1-" 
                      << instance.customers.size() << "). Skipping." << std::endl;
            continue;  // Skip invalid customer
        }
        
        int customer_idx = customer_id - 1;
        int vehicle_id = assignment[customer_idx];

        if (vehicle_id < 1 || vehicle_id > num_technicians_ + num_drones_)
        {
            throw std::runtime_error("Error: Invalid vehicle ID in assignment.");
        } 
        if (vehicle_id <= num_technicians_)
        {
            // Technician
            auto& route = solution.truckRoutes[vehicle_id - 1];
            route.customers.push_back(customer_id);
        }
        else
        {
            if (instance.customers[customer_idx].isStaffOnly) {
                // Staff-only customers MUST use truck, not drone
                auto& truckRoute = solution.truckRoutes[0];
                truckRoute.customers.push_back(customer_id);
                continue;  // Skip to next customer - important
            }
            // Drone
            int drone_index = vehicle_id - num_technicians_ - 1;
            auto& droneTrips = solution.droneRoutes[drone_index];
            bool createNewTrip = false;

            if (droneTrips.empty()) {
                createNewTrip = true;
            } else {
                // 1. Kiểm tra Tải trọng
                double currentLoad = 0;
                for (int c : droneTrips.back().customers) {
                    currentLoad += instance.customers[c - 1].demand;
                }
                double newLoad = currentLoad + instance.customers[customer_idx].demand;
                
                if (newLoad > instance.droneParams[drone_index].maxCapacity) {
                    createNewTrip = true;
                } 
                else {
                    // 2. Kiểm tra Năng lượng (Giả lập thêm khách vào để tính thử)
                    Route tempTrip = droneTrips.back();
                    tempTrip.customers.push_back(customer_id);
                    
                    // Lưu ý: evaluateSingleDroneTrip trả về kJ
                    double neededEnergy = evaluateSingleDroneTrip(tempTrip, drone_index);
                    
                    if (neededEnergy > instance.droneParams[drone_index].maxEnergy) {
                        createNewTrip = true;
                    } else {
                        // 3. Kiểm tra Thời gian bay
                        double flightTime = calculateTripFlightTime(tempTrip, drone_index);
                        
                        if (flightTime > instance.droneParams[drone_index].maxFlightTime) {
                            createNewTrip = true;
                        }
                    }
                }
            }

            if (createNewTrip)
            {
                // Tạo chuyến mới và kiểm tra feasibility
                Route newTrip;
                newTrip.customers.push_back(customer_id);
                
                // Kiểm tra xem trip mới có vi phạm ràng buộc không
                double flightTime = calculateTripFlightTime(newTrip, drone_index);
                double neededEnergy = evaluateSingleDroneTrip(newTrip, drone_index);
                
                bool isFlightTimeViolation = (flightTime > instance.droneParams[drone_index].maxFlightTime);
                bool isEnergyViolation = (neededEnergy > instance.droneParams[drone_index].maxEnergy);
                bool isCapacityViolation = (instance.customers[customer_idx].demand > instance.droneParams[drone_index].maxCapacity);
                
                if (isFlightTimeViolation || isEnergyViolation || isCapacityViolation) {
                    // HIGH-TRUCK OPTIMIZATION: Try other drones before falling back to trucks
                    bool assignedToOtherDrone = false;
                    
                    if (inst.numTrucks >= 30) {
                        // For high-truck cases: try ALL other drones first
                        for (int alt_drone = 0; alt_drone < num_drones_; alt_drone++) {
                            if (alt_drone == drone_index) continue;  // Skip current drone
                            
                            // Test if this customer can be served by alternative drone
                            Route testTrip;
                            testTrip.customers.push_back(customer_id);
                            
                            double altFlightTime = calculateTripFlightTime(testTrip, alt_drone);
                            double altEnergy = evaluateSingleDroneTrip(testTrip, alt_drone);
                            double altDemand = instance.customers[customer_idx].demand;
                            
                            bool altTimeOK = (altFlightTime <= instance.droneParams[alt_drone].maxFlightTime);
                            bool altEnergyOK = (altEnergy <= instance.droneParams[alt_drone].maxEnergy);
                            bool altCapacityOK = (altDemand <= instance.droneParams[alt_drone].maxCapacity);
                            
                            if (altTimeOK && altEnergyOK && altCapacityOK) {
                                // Found a feasible alternative drone!
                                solution.droneRoutes[alt_drone].push_back(testTrip);
                                assignedToOtherDrone = true;
                                break;
                            }
                        }
                    }
                    
                    if (!assignedToOtherDrone) {
                        // No drone can serve → fallback to truck (least loaded)
                        int bestTruck = 0;
                        int minCustomers = solution.truckRoutes[0].customers.size();
                        for (int t = 1; t < instance.numTrucks; t++) {
                            if (solution.truckRoutes[t].customers.size() < minCustomers) {
                                minCustomers = solution.truckRoutes[t].customers.size();
                                bestTruck = t;
                            }
                        }
                        solution.truckRoutes[bestTruck].customers.push_back(customer_id);
                    }
                } else {
                    // Trip hợp lệ, thêm vào drone
                    droneTrips.push_back(newTrip);
                }
            }
            else
            {
                // Thêm vào chuyến hiện tại
                droneTrips.back().customers.push_back(customer_id);
            }
        }
        // routes_by_vehicle[vehicle_id].push_back(customer_id);
    }

    // In ra các tuyến đường
    for (const auto &pair : routes_by_vehicle)
    {
        int vehicle_id = pair.first;
        const auto &customer_list = pair.second;

        // Xác định loại phương tiện dựa trên ID
        if (vehicle_id <= num_technicians_)
        { // Đây là KTV
            std::cout << "Technician " << vehicle_id << " (1 trip): Depot -> ";
            for (int c_id : customer_list)
            {
                std::cout << "C" << c_id << " -> ";
            }
            std::cout << "Depot\n";
        }
        else
        { // Đây là Drone
            int trip_count = 0;
            std::vector<int> current_trip;
            for (int c_id : customer_list)
            {
                // if (current_trip.size() == DRONE_TRIP_CAPACITY)
                // {
                    std::cout << "Drone " << vehicle_id << " (Trip " << ++trip_count << "): Depot -> ";
                    for (int trip_c_id : current_trip)
                        std::cout << "C" << trip_c_id << " -> ";
                    std::cout << "Depot\n";
                    current_trip.clear();
                // }
                current_trip.push_back(c_id);
            }
            if (!current_trip.empty())
            {
                std::cout << "Drone " << vehicle_id << " (Trip " << ++trip_count << "): Depot -> ";
                for (int trip_c_id : current_trip)
                    std::cout << "C" << trip_c_id << " -> ";
                std::cout << "Depot\n";
            }
        }
    }

    // Evaluate final solution
    evaluator.evaluate(solution);
    
    return solution;
}

double Decoder::evaluateSingleDroneTrip(const Route& trip, int droneId) {
    // TÍNH NĂNG LƯỢNG TIÊU THỤ CỦA DRONE
    // Energy = (β * Load + γ) * flightTime

    if (trip.isEmpty()) return 0;
    
    const auto& droneParam = instance.droneParams[droneId];
    double totalEnergy = 0;
    double currentLoad = 0;
    const int height = 50;

    double takeoffSpeed = droneParam.takeoffSpeed;
    double landingSpeed = droneParam.landingSpeed;

    double takeoffTime = takeoffSpeed != 0 ? height / takeoffSpeed : 0;
    double landingTime = landingSpeed != 0 ? height / landingSpeed : 0;

    for (int custId : trip.customers) {
        currentLoad += instance.customers[custId - 1].demand;
    }
    
    int prevNode = 0;
    
    for (int custId : trip.customers) {
        double distance = instance.getDistance(prevNode, custId);
        double travelTime = distance / droneParam.cruiseSpeed;
        
        double power = droneParam.beta * currentLoad + droneParam.gamma;
        double energy = power * (takeoffTime + travelTime + landingTime);
        totalEnergy += energy;
        
        // Sau khi lấy mẫu, tải trọng giảm
        currentLoad -= instance.customers[custId - 1].demand;
        prevNode = custId;
    }
    
    // Quay về depot (không có tải)
    double distance = instance.getDistance(prevNode, 0);
    double travelTime = distance / droneParam.cruiseSpeed;
    double power = droneParam.beta * currentLoad + droneParam.gamma;
    totalEnergy += power * (takeoffTime + travelTime + landingTime);
    
    return totalEnergy / 1000.0;    // Chuyển sang kJ
}

double Decoder::calculateTripFlightTime(const Route& trip, int droneId) {
    // TÍNH THỜI GIAN BAY CỦA DRONE TRIP
    if (trip.isEmpty()) return 0;
    
    const auto& droneParam = instance.droneParams[droneId];
    const int height = 50;
    double takeoffSpeed = droneParam.takeoffSpeed;
    double landingSpeed = droneParam.landingSpeed;
    double cruiseSpeed = droneParam.cruiseSpeed;
    
    double takeoffTime = takeoffSpeed != 0 ? height / takeoffSpeed : 0;
    double landingTime = landingSpeed != 0 ? height / landingSpeed : 0;
    
    double totalTime = 0;
    int prevNode = 0;
    
    for (int custId : trip.customers) {
        double distance = instance.getDistance(prevNode, custId);
        double travelTime = distance / cruiseSpeed;
        totalTime += takeoffTime + travelTime + landingTime;
        prevNode = custId;
    }
    
    // Quay về depot
    double distance = instance.getDistance(prevNode, 0);
    totalTime += takeoffTime + (distance / cruiseSpeed) + landingTime;
    
    return totalTime;
}