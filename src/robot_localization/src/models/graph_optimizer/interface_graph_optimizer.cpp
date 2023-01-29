/*
 * @Description: 
 */

#include "../../../include/models/graph_optimizer/graph_optimizer_interface.hpp"

namespace robot_localization {

void InterfaceGraphOptimizer::SetMaxIterationsNum(int max_iterations_num) 
{
    max_iterations_num_ = max_iterations_num;
}

}