#ifndef __FIXED_POINT_TASK_H
#define __FIXED_POINT_TASK_H

#include <array>

namespace YXWestimation {

#define FIXED_POINT_ENABLE 0

using PostureAngle = std::array<std::array<double, 7>, 7>;

typedef struct FixedPointStatus {
	bool    IsSend;
	int     PostureStatus;
} FixedPointStatus;

extern FixedPointStatus arm_posture;

void send_fixed_point_data(int posture_id, PostureAngle& left, PostureAngle& right);




#endif

}

