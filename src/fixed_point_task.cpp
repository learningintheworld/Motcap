#include "fixed_point_task.h"
#include <iostream>

namespace YXWestimation {

FixedPointStatus arm_posture = {0, 0};
PostureAngle left_arm_angle = {{
    {0, 0, 0, 0, 0, 0, 0},
    {1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7},
    {1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7},
    {1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7},
    {1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7},
}};
PostureAngle right_arm_angle = {{
    {0, 0, 0, 0, 0, 0, 0},
    {1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7},
    {1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7},
    {1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7},
    {1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7},
}};

void send_fixed_point_data(int posture_id){

}

}