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
                
                if (newLoad > instance.droneParams.maxCapacity) {
                    createNewTrip = true;
                } 
                else {
                    // 2. Kiểm tra Năng lượng (Giả lập thêm khách vào để tính thử)
                    Route tempTrip = droneTrips.back();
                    tempTrip.customers.push_back(customer_id);
                    
                    // Lưu ý: evaluateSingleDroneTrip trả về kJ
                    double neededEnergy = evaluateSingleDroneTrip(tempTrip, drone_index);
                    
                    if (neededEnergy > instance.droneParams.maxEnergy) {
                        createNewTrip = true;
                    }
                }
            }

            if (createNewTrip)
            {
                // Tạo chuyến mới
                Route newTrip;
                newTrip.customers.push_back(customer_id);
                
                // Check ngay xem chuyến mới có feasible không (đặc biệt với khách quá xa hoặc quá nặng)
                // Nếu cần thiết thì xử lý infeasible tại đây
                
                droneTrips.push_back(newTrip);
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
    
    double totalEnergy = 0;
    double currentLoad = 0;
    const int height = 50;

    double takeoffSpeed = instance.droneParams.takeoffSpeed;
    double landingSpeed = instance.droneParams.landingSpeed;

    double takeoffTime = takeoffSpeed != 0 ? height / takeoffSpeed : 0;
    double landingTime = landingSpeed != 0 ? height / landingSpeed : 0;

    for (int custId : trip.customers) {
        currentLoad += instance.customers[custId - 1].demand;
    }
    
    int prevNode = 0;
    
    for (int custId : trip.customers) {
        double distance = instance.getDistance(prevNode, custId);
        double travelTime = distance / instance.droneParams.cruiseSpeed;
        
        double power = instance.droneParams.beta * currentLoad + instance.droneParams.gamma;
        double energy = power * (takeoffTime + travelTime + landingTime);
        totalEnergy += energy;
        
        // Sau khi lấy mẫu, tải trọng giảm
        currentLoad -= instance.customers[custId - 1].demand;
        prevNode = custId;
    }
    
    // Quay về depot (không có tải)
    double distance = instance.getDistance(prevNode, 0);
    double travelTime = distance / instance.droneParams.cruiseSpeed;
    double power = instance.droneParams.beta * currentLoad + instance.droneParams.gamma;
    totalEnergy += power * (takeoffTime + travelTime + landingTime);
    
    return totalEnergy / 1000.0;    // Chuyển sang kJ
}