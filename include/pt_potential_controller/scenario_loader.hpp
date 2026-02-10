#ifndef PT_POTENTIAL_PLANNER__SCENARIO_PARSER_HPP
#define PT_POTENTIAL_PLANNER__SCENARIO_PARSER_HPP

#include <scenario.hpp>

#include <yaml-cpp/yaml.h>

namespace pt_potential_controller {

    ScenarioPtr load_from_file(std::string path);

}

#endif