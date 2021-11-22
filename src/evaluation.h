#include <ros/ros.h>

#include <iostream>
#include <filesystem>
#include <fstream>
#include <string>

class Evaluation {
    std::string file_path;
    std::vector<float> radius_errors;
public:
    Evaluation();
    void clear_file();
    int dump_errors(float error);
    void eval_iter(float min_radius);
};