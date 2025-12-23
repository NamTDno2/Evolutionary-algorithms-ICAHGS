#include "InputReader.h"
#include <algorithm>
#include <fstream>
#include <sstream>
#include <iostream>
#include <cctype>
#include "../include/json.hpp"

using namespace std;
using json = nlohmann::json;

DroneParams readDroneParams(int index)
{
    DroneParams droneParams;
    std::ifstream file("config/drone_linear_config.json");
    if (!file.is_open())
    {
        std::cerr << "Không thể mở file JSON!\n";
        return droneParams;
    }

    json data;
    file >> data;

    if (data.contains(std::to_string(index)))
    {
        auto item = data[std::to_string(index)];
        droneParams.takeoffSpeed = item["takeoffSpeed [m/s]"].get<double>();
        droneParams.cruiseSpeed = item["cruiseSpeed [m/s]"].get<double>();
        droneParams.landingSpeed = item["landingSpeed [m/s]"].get<double>();
        droneParams.maxCapacity = item["capacity [kg]"].get<double>();
        droneParams.maxEnergy = item["batteryPower [Joule]"].get<double>() / 1000.0;  // Convert J to kJ
        droneParams.beta = item["beta(w/kg)"].get<double>();
        droneParams.gamma = item["gama(w)"].get<double>();
    }

    return droneParams;
}

TruckParams readTruckParams() {
    TruckParams truckParams;
    std::ifstream file("config/Truck_config.json");
    if (!file.is_open())
    {
        std::cerr << "Không thể mở file JSON!\n";
        return truckParams;
    }

    json data;
    file >> data;

    truckParams.maxSpeed = data["V_max (m/s)"].get<double>();

    auto hours = data["T (hour)"];
    for (auto& [key, value] : hours.items()) {
        int startHour = 0, endHour = 0;

        size_t pos = key.find('-');
        if (pos != std::string::npos) {
            startHour = std::stoi(key.substr(0, pos));
            endHour = std::stoi(key.substr(pos + 1));
        }

        int startSec = startHour * 3600;
        int endSec   = endHour * 3600;

        double factor = value.get<double>();

        truckParams.timeIntervals.emplace_back(startSec, endSec, factor);
    }

    return truckParams;
}

bool InputReader::readInstance(const string& filename, Instance& instance) {
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Cannot open file: " << filename << endl;
        return false;
    }
    
    string line;
    
    // Read number_staff (giả sử dòng có dạng "key value")
    if (getline(file, line)) {
        auto parts = split(line, ' ');
        if (parts.size() >= 2) {
            instance.numTrucks = stoi(parts[1]);
        }
    }
    
    // Read number_drone
    if (getline(file, line)) {
        auto parts = split(line, ' ');
        if (parts.size() >= 2) {
            instance.numDrones = stoi(parts[1]);
        }
    }
    
    // Read droneLimitationFightTime
    double tempMaxFlightTime = 3600.0;  // Default value
    if (getline(file, line)) {
        auto parts = split(line, ' ');
        if (parts.size() >= 2) {
            tempMaxFlightTime = stod(parts[1]);
        }
    }
    
    // Read Customers count
    int numCustomers = 0;
    if (getline(file, line)) {
        auto parts = split(line, ' ');
        if (parts.size() >= 2) {
            numCustomers = stoi(parts[1]);
        }
    }
    
    // Skip header line
    getline(file, line);
    
    // Read customers
    for (int i = 0; i < numCustomers; i++) {
        if (getline(file, line)) {
            auto parts = split(line, ' ');
            if (parts.size() >= 6) {
                Customer cust;
                cust.id = i + 1;
                
                cust.x = stod(parts[0]);
                cust.y = stod(parts[1]);
                cust.demand = stod(parts[2]);
                cust.isStaffOnly = (stoi(parts[3]) == 1);
                cust.serviceTimeTruck = stod(parts[4]);
                cust.serviceTimeDrone = stod(parts[5]);
                
                instance.customers.push_back(cust);
            }
        }
    }
    
    // Read Beta (skip, will use from config)
    if (getline(file, line)) {
        // Skip "Beta" label
    }
    if (getline(file, line)) {
        // Skip beta value - using from drone config file instead
    }
    
    file.close();
    
    
    // Read drone params for all drones (homogeneous fleet)
    instance.droneParams.resize(instance.numDrones);
    for (int i = 0; i < instance.numDrones; i++) {
        // USE ONLY DRONE TYPE 4: low-speed-high-range
        // (takeoffSpeed=7.8232, cruiseSpeed=15.6464, batteryPower=562990J)
        int droneType = 4;  
        instance.droneParams[i] = readDroneParams(droneType);
        instance.droneParams[i].maxFlightTime = tempMaxFlightTime;
    }
    
    instance.truckParams = readTruckParams();
    
    return true;
}

vector<string> InputReader::split(const string& s, char delimiter) {
    vector<string> tokens;
    string token;
    istringstream tokenStream(s);
    
    // Sử dụng toán tử trích xuất (>>) để xử lý mọi loại khoảng trắng
    while (tokenStream >> token) {
        tokens.push_back(token);
    }
    return tokens;
}

void InputReader::trim(string& s) {
    s.erase(s.begin(), find_if(s.begin(), s.end(), [](unsigned char ch) {
        return !isspace(ch);
    }));
    s.erase(find_if(s.rbegin(), s.rend(), [](unsigned char ch) {
        return !isspace(ch);
    }).base(), s.end());
}
