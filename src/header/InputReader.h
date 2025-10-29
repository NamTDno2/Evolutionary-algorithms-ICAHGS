#ifndef INPUTREADER_H
#define INPUTREADER_H

#include "DataStructures.h"
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

class InputReader {
public:
    static bool readInstance(const std::string& filename, Instance& instance);
    
private:
    static std::vector<std::string> split(const std::string& s, char delimiter);
    static void trim(std::string& s);
};

#endif // INPUTREADER_H
