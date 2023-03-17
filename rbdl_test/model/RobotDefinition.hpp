#ifndef ROBOT_DEFINITION
#define ROBOT_DEFINITION

#include "common_utils/common.hpp"

namespace robot{
    constexpr int num_q = 14;
    constexpr int num_qdot = 13;
    constexpr int num_act_joint = 7; // 行动联合
    constexpr int num_virtual = 2;  //虚拟
};

namespace robot_link{
    constexpr int base_link = 0;
    constexpr int link1 = 1;
    constexpr int link2 = 2;
    constexpr int link3 = 3;
    constexpr int link4 = 4;
    constexpr int link5 = 5;
    constexpr int link6 = 6;
    constexpr int link7 = 7;
    constexpr int flange = 8;
};
namespace robot_joint{
    constexpr int virtual_X = 0;
    constexpr int virtual_Y = 1;
    constexpr int virtual_Z = 2;
    constexpr int virtual_Rx = 3;
    constexpr int virtual_Ry = 4;
    constexpr int virtual_Rz = 5;

    constexpr int joint1 = 6;
    constexpr int joint2 = 7;
    constexpr int joint3 = 8;
    
    constexpr int joint4 = 9;
    constexpr int joint5 = 10;
    constexpr int joint6 = 11;
    
    constexpr int joint7 = 12;

    constexpr int virtual_Rw = 13;
};
extern const char* link_name[];

#endif

