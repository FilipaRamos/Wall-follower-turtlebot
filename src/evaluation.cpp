#include "evaluation.h"

Evaluation::Evaluation() {}

int Evaluation::dump_errors(float error) {
    std::ofstream file;
    
    file.open("./src/turtlebot3_wall/eval/range_errors.txt", std::ios_base::app);

    file << error << "\n";
    file.close();
    
    return 0;
}

void Evaluation::eval_iter(float min_radius) {
    float error = min_radius - 1.0;
    radius_errors.push_back(error);
    dump_errors(error);
}