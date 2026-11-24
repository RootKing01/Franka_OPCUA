#ifndef FRANKA_OPCUA_BRIDGE__EVENTS__FRANKA_DECISION_HPP_
#define FRANKA_OPCUA_BRIDGE__EVENTS__FRANKA_DECISION_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace franka_opcua_bridge
{

struct FrankaDecisionRequest
{
    std::uint64_t request_id;
    std::uint64_t decision_id;

    std::string description;

    std::vector<int32_t> option_codes;
    std::vector<std::string> option_labels;

    std::vector<std::string> parameter_names;
    std::vector<std::string> parameter_values;
};

struct FrankaDecisionResponse
{
    std::uint64_t request_id;
    std::uint64_t decision_id;

    int32_t decision_code;
};

}

#endif