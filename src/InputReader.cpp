#include "InputReader.h"
#include <algorithm>

bool InputReader::readInstance(const std::string& filename, Instance& instance) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Cannot open file: " << filename << std::endl;
        return false;
    }
    
    std::string line;
    
    // Read number_staff
    if (std::getline(file, line)) {
        auto parts = split(line, ' ');
        if (parts.size() >= 2) {
            instance.numTrucks = std::stoi(parts);
        }
    }
    
    // Read number_drone
    if (std::getline(file, line)) {
        auto parts = split(line, ' ');
        if (parts.size() >= 2) {
            instance.numDrones = std::stoi(parts);
        }
    }
    
    // Read droneLimitationFightTime
    if (std::getline(file, line)) {
        auto parts = split(line, ' ');
        if (parts.size() >= 2) {
            instance.droneParams.maxFlightTime = std::stod(parts);
        }
    }
    
    // Read Customers count
    int numCustomers = 0;
    if (std::getline(file, line)) {
        auto parts = split(line, ' ');
        if (parts.size() >= 2) {
            numCustomers = std::stoi(parts);
        }
    }
    
    // Skip header line
    std::getline(file, line);
    
    // Read customers
    for (int i = 0; i < numCustomers; i++) {
        if (std::getline(file, line)) {
            auto parts = split(line, ' ');
            if (parts.size() >= 6) {
                Customer cust;
                cust.id = i + 1;
                cust.x = std::stod(parts);
                cust.y = std::stod(parts);
                cust.demand = std::stod(parts);
                cust.isStaffOnly = (std::stoi(parts) == 1);
                cust.serviceTimeTruck = std::stod(parts);
                cust.serviceTimeDrone = std::stod(parts);
                
                instance.customers.push_back(cust);
            }
        }
    }
    
    // Read Beta
    if (std::getline(file, line)) {
        // Skip "Beta" label
    }
    if (std::getline(file, line)) {
        instance.droneParams.beta = std::stod(line);
    }
    
    file.close();
    
    // Set default parameters (should be read from config)
    instance.droneParams.maxCapacity = 5.0;  // kg
    instance.droneParams.maxEnergy = 500.0;  // kJ
    instance.droneParams.takeoffSpeed = 5.0;  // m/s
    instance.droneParams.cruiseSpeed = 15.0;  // m/s
    instance.droneParams.landingSpeed = 5.0;  // m/s
    instance.droneParams.gamma = 100.0;  // W
    
    instance.truckParams.maxSpeed = 20.0;  // m/s
    
    // Create simple time intervals (can be extended)
    instance.truckParams.timeIntervals.push_back(TimeInterval(0, 3600, 0.8));
    instance.truckParams.timeIntervals.push_back(TimeInterval(3600, 7200, 1.0));
    instance.truckParams.timeIntervals.push_back(TimeInterval(7200, 14400, 0.8));
    
    return true;
}

std::vector<std::string> InputReader::split(const std::string& s, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    
    while (std::getline(tokenStream, token, delimiter)) {
        trim(token);
        if (!token.empty()) {
            tokens.push_back(token);
        }
    }
    return tokens;
}

void InputReader::trim(std::string& s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch) {
        return !std::isspace(ch);
    }));
    s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) {
        return !std::isspace(ch);
    }).base(), s.end());
}
