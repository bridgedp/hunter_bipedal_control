/********************************************************************************
Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#include "legged_interface/foot_planner/MultiCubicSpline.h"
#include "ocs2_core/misc/Numerics.h"
#include "legged_common/output_color.h"

namespace ocs2
{
namespace legged_robot
{
using namespace output_color;

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MultiCubicSpline::MultiCubicSpline(std::vector<CubicSpline::Node> nodes) : nodes_(std::move(nodes))
{
  cubic_splines_.reserve(nodes_.size() - 1);
  for (int i = 0; i < nodes_.size() - 1; i++)
  {
    cubic_splines_.emplace_back(nodes_[i], nodes_[i + 1]);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t MultiCubicSpline::position(scalar_t time) const
{
  for (int i = 0; i < nodes_.size() - 1; i++)
  {
    if (time >= nodes_[i].time && time < nodes_[i + 1].time)
    {
      return cubic_splines_[i].position(time);
    }
  }
  if (numerics::almost_eq(time, nodes_.back().time))
    return cubic_splines_.back().position(time);
  if (time < nodes_.front().time)
    return cubic_splines_.front().position(time);
  if (time > nodes_.back().time)
    return cubic_splines_.back().position(time);
  return 0;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t MultiCubicSpline::velocity(scalar_t time) const
{
  for (int i = 0; i < nodes_.size() - 1; i++)
  {
    if (time >= nodes_[i].time && time < nodes_[i + 1].time)
    {
      return cubic_splines_[i].velocity(time);
    }
  }
  if (numerics::almost_eq(time, nodes_.back().time))
    return cubic_splines_.back().velocity(time);
  if (time < nodes_.front().time)
    return cubic_splines_.front().velocity(time);
  if (time > nodes_.back().time)
    return cubic_splines_.back().velocity(time);
  return 0;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t MultiCubicSpline::acceleration(scalar_t time) const
{
  for (int i = 0; i < nodes_.size() - 1; i++)
  {
    if (time >= nodes_[i].time && time < nodes_[i + 1].time)
    {
      return cubic_splines_[i].acceleration(time);
    }
  }
  if (numerics::almost_eq(time, nodes_.back().time))
    return cubic_splines_.back().acceleration(time);
  if (time < nodes_.front().time)
    return cubic_splines_.front().acceleration(time);
  if (time > nodes_.back().time)
    return cubic_splines_.back().acceleration(time);
  return 0;
}

}  // namespace legged_robot
}  // namespace ocs2
