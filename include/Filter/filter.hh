#pragma once

#include <Eigen/Core>

#include "SystemModel/systemmodel.hh"
#include "ObservationModel/observationmodel.hh"

namespace rbest
{

  template<typename SYSTEM_MODEL_TYPE, typename OBSERVATION_MODEL_TYPE>
  class Filter
  {
  public:
    using StateVector       = typename SYSTEM_MODEL_TYPE::StateVector;
    using ControlVector     = typename SYSTEM_MODEL_TYPE::ControlVector;
    using ObservationVector = typename OBSERVATION_MODEL_TYPE::ObservationVector;

    using SystemModelType      = SYSTEM_MODEL_TYPE;
    using ObservationModelType = OBSERVATION_MODEL_TYPE;
    
  public:
    virtual void predict(SystemModelType const& systemModel, ControlVector const& control) = 0;

    virtual void update(ObservationModelType const& observationModel, ObservationVector const& observation) = 0;

    virtual void init(StateVector state) = 0;
    
    virtual StateVector getState() const = 0;
  };
}
