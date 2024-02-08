/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

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

#include "legged_interface/constraint/ZeroForceConstraint.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>

namespace ocs2
{
namespace legged_robot
{
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ZeroForceConstraint::ZeroForceConstraint(const SwitchedModelReferenceManager& referenceManager,
                                         size_t contactPointIndex, CentroidalModelInfo info)
  : StateInputConstraint(ConstraintOrder::Linear)
  , referenceManagerPtr_(&referenceManager)
  , contactPointIndex_(contactPointIndex)
  , info_(std::move(info))
{
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool ZeroForceConstraint::isActive(scalar_t time) const
{
  return !referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t ZeroForceConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                       const PreComputation& preComp) const
{
  vector_t force(getNumConstraints(time));
  force << centroidal_model::getContactForces(input, contactPointIndex_, info_);
  return force;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation ZeroForceConstraint::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                              const vector_t& input,
                                                                              const PreComputation& preComp) const
{
  VectorFunctionLinearApproximation approx;
  approx.f = getValue(time, state, input, preComp);
  approx.dfdx = matrix_t::Zero(getNumConstraints(time), state.size());
  approx.dfdu = matrix_t::Zero(getNumConstraints(time), input.size());
  const size_t contactForceIndex = 3 * contactPointIndex_;
  const size_t contactWrenchIndex =
      3 * info_.numThreeDofContacts + 6 * (contactPointIndex_ - info_.numThreeDofContacts);
  const size_t startRow = (contactPointIndex_ < info_.numThreeDofContacts) ? contactForceIndex : contactWrenchIndex;
  approx.dfdu.middleCols(startRow, getNumConstraints(time)).diagonal() = vector_t::Ones(getNumConstraints(time));
  return approx;
}

}  // namespace legged_robot
}  // namespace ocs2
