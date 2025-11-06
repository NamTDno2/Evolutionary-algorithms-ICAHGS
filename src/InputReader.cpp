#include "InputReader.h"
#include <algorithm>
#include <fstream>
#include <sstream>
#include <iostream>
#include <cctype>

using namespace std;

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
    if (getline(file, line)) {
        auto parts = split(line, ' ');
        if (parts.size() >= 2) {
            instance.droneParams.maxFlightTime = stod(parts[1]);
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
    
    // Read Beta
    if (getline(file, line)) {
        // Skip "Beta" label
    }
    if (getline(file, line)) {
        instance.droneParams.beta = stod(line);
    }
    
    file.close();
    
    // Set default parameters
    instance.droneParams.maxCapacity = 5.0;
    instance.droneParams.maxEnergy = 500.0;
    instance.droneParams.takeoffSpeed = 5.0;
    instance.droneParams.cruiseSpeed = 15.0;
    instance.droneParams.landingSpeed = 5.0;
    instance.droneParams.gamma = 100.0;
    
    instance.truckParams.maxSpeed = 20.0;
    
    // Create simple time intervals
    instance.truckParams.timeIntervals.push_back(TimeInterval(0, 3600, 0.8));
    instance.truckParams.timeIntervals.push_back(TimeInterval(3600, 7200, 1.0));
    instance.truckParams.timeIntervals.push_back(TimeInterval(7200, 14400, 0.8));
    
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
