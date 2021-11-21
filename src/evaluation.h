#include <ros/ros.h>

#include <iostream>
#include <filesystem>
#include <fstream>
#include <string>

class Evaluation {
    std::string folder;
    std::vector<float> radius_errors;
public:
    Evaluation();
    int dump_errors(float error);
    void eval_iter(float min_radius);
};