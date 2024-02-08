/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_core/misc/Numerics.h>

#include "legged_interface/LeggedRobotPreComputation.h"

namespace ocs2
{
namespace legged_robot
{
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LeggedRobotPreComputation::LeggedRobotPreComputation(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                                                     const SwingTrajectoryPlanner& swingTrajectoryPlanner,
                                                     ModelSettings settings)
  : pinocchioInterface_(std::move(pinocchioInterface))
  , info_(std::move(info))
  , swingTrajectoryPlannerPtr_(&swingTrajectoryPlanner)
  , mappingPtr_(new CentroidalModelPinocchioMapping(info_))
  , settings_(std::move(settings))
{
  eeNormalVelConConfigs_.resize(info_.numThreeDofContacts);
  eeXYRefConConfigs_.resize(info_.numThreeDofContacts);
  eeXYLimitConConfigs_.resize(info_.numThreeDofContacts);
  mappingPtr_->setPinocchioInterface(pinocchioInterface_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LeggedRobotPreComputation::LeggedRobotPreComputation(const LeggedRobotPreComputation& rhs)
  : pinocchioInterface_(rhs.pinocchioInterface_)
  , info_(rhs.info_)
  , swingTrajectoryPlannerPtr_(rhs.swingTrajectoryPlannerPtr_)
  , mappingPtr_(rhs.mappingPtr_->clone())
  , settings_(rhs.settings_)
{
  eeNormalVelConConfigs_.resize(rhs.eeNormalVelConConfigs_.size());
  eeXYRefConConfigs_.resize(rhs.eeXYRefConConfigs_.size());
  eeXYLimitConConfigs_.resize(rhs.eeXYLimitConConfigs_.size());
  mappingPtr_->setPinocchioInterface(pinocchioInterface_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotPreComputation::request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u)
{
  if (!request.containsAny(Request::Cost + Request::Constraint + Request::SoftConstraint))
  {
    return;
  }

  auto eeNormalVelConConfig = [&](size_t footIndex) {
    EndEffectorLinearConstraint::Config config;
    config.b = (vector_t(1) << -swingTrajectoryPlannerPtr_->getZvelocityConstraint(footIndex, t)).finished();
    config.Av = (matrix_t(1, 3) << 0.0, 0.0, 1.0).finished();
    if (!numerics::almost_eq(settings_.positionErrorGain, 0.0))
    {
      config.b(0) -= settings_.positionErrorGain * swingTrajectoryPlannerPtr_->getZpositionConstraint(footIndex, t);
      config.Ax = (matrix_t(1, 3) << 0.0, 0.0, settings_.positionErrorGain).finished();
    }
    return config;
  };

  // lambda to set config for xy velocity constraints
  auto eeXYRefConConfig = [&](size_t footIndex) {
    EndEffectorLinearConstraint::Config config;
    config.b = (vector_t(2) << -swingTrajectoryPlannerPtr_->getXvelocityConstraint(footIndex, t),
                -swingTrajectoryPlannerPtr_->getYvelocityConstraint(footIndex, t))
                   .finished();
    config.Av = (matrix_t(2, 3) << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0).finished();
    config.b(0) -= 3 * swingTrajectoryPlannerPtr_->getXpositionConstraint(footIndex, t);
    config.b(1) -= 3 * swingTrajectoryPlannerPtr_->getYpositionConstraint(footIndex, t);
    config.Ax = (matrix_t(2, 3) << 3, 0.0, 0.0, 0.0, 3, 0.0).finished();
    return config;
  };

  auto eeXYLimitConConfig = [&](size_t footIndex) {
    const scalar_t k1 = 20;
    const scalar_t k2 = 20;
    const auto t03 = swingTrajectoryPlannerPtr_->getSwingStartStopTime(footIndex, t);
    const auto t0 = t03.front();
    const auto t3 = t03.back();
    const scalar_t t1 = t0;
    const scalar_t t2 = t3 - 0.1;
    const scalar_t weight = 10;

    EndEffectorLinearConstraint::Config config;
    config.b =
        (vector_t(4) << ((t >= t0 && t <= t2) ? weight * k1 * (t - t0) : 0.0),
         ((t >= t0 && t <= t2) ? weight * k1 * (t - t0) : 0.0), ((t >= t2 && t <= t3) ? weight * k2 * (t - t3) : 0.0),
         ((t >= t2 && t <= t3) ? weight * k2 * (t - t3) : 0.0))
            .finished();
    config.Av = (matrix_t(4, 3) << ((t >= t0 && t <= t2) ? -weight * 1.0 : 0.0), 0.0, 0.0, 0.0,
                 ((t >= t0 && t <= t2) ? -weight * 1.0 : 0.0), 0.0, ((t >= t2 && t <= t3) ? weight * 1.0 : 0.0), 0.0,
                 0.0, 0.0, ((t >= t2 && t <= t3) ? weight * 1.0 : 0.0), 0.0)
                    .finished();
    return config;
  };

  if (request.contains(Request::Constraint))
  {
    for (size_t i = 0; i < info_.numThreeDofContacts; i++)
    {
      eeNormalVelConConfigs_[i] = eeNormalVelConConfig(i);
      eeXYRefConConfigs_[i] = eeXYRefConConfig(i);
    }
  }

  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  vector_t q = mappingPtr_->getPinocchioJointPosition(x);
  if (request.contains(Request::Approximation))
  {
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
    pinocchio::updateGlobalPlacements(model, data);
    pinocchio::computeJointJacobians(model, data);

    updateCentroidalDynamics(pinocchioInterface_, info_, q);
    vector_t v = mappingPtr_->getPinocchioJointVelocity(x, u);
    updateCentroidalDynamicsDerivatives(pinocchioInterface_, info_, q, v);
  }
  else
  {
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
  }
}

}  // namespace legged_robot
}  // namespace ocs2
