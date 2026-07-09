#pragma once
#include <string>

class EnvironmentRecord {
public:
    // Write a key=value environment record to `filename`.
    // physics_list and seed are passed from the current run configuration.
    static void Write(const std::string& filename,
                      const std::string& physics_list,
                      long seed);
};
