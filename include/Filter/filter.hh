#pragma once

#include <Eigen/Core>

#include "SystemModel/systemmodel.hh"
#include "ObservationModel/observationmodel.hh"

namespace rbest
{

  template<typename VECS_TYPE, int STATE_DIM, int CONTROL_DIM, int OBSERVATION_DIM>
  class Filter
  {
  public:
    using StateVector       = Eigen::Matrix<VECS_TYPE, STATE_DIM, 1>;
    using ControlVector     = Eigen::Matrix<VECS_TYPE, CONTROL_DIM, 1>;
    using ObservationVector = Eigen::Matrix<VECS_TYPE, OBSERVATION_DIM, 1>;

    using SystemModelType      = SystemModel<VECS_TYPE, STATE_DIM, CONTROL_DIM>;
    using ObservationModelType = ObservationModel<VECS_TYPE, STATE_DIM, OBSERVATION_DIM>;
    
  public:
    virtual void predict(SystemModelType const& systemModel, ControlVector const& control) = 0;

    virtual void update(ObservationModelType const& observationModel, ObservationVector const& observation) = 0;
  };
}
