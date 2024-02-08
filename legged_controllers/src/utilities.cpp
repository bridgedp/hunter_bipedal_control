/********************************************************************************
Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#include "legged_controllers/utilities.h"

namespace legged
{
using namespace ocs2;

std_msgs::Float64MultiArray createFloat64MultiArrayFromVector(const vector_t& data)
{
  std_msgs::Float64MultiArray msg;
  msg.data.resize(data.size());
  vector_t::Map(&msg.data[0], data.size()) = data;
  return msg;
}

}  // namespace legged