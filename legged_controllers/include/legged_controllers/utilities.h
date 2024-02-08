/********************************************************************************
Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#pragma once

#include <ocs2_core/Types.h>
#include "std_msgs/Float64MultiArray.h"

namespace legged
{
using namespace ocs2;

std_msgs::Float64MultiArray createFloat64MultiArrayFromVector(const vector_t& data);

}  // namespace legged