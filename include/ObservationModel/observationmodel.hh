#pragma once

#include <Eigen/Core>

namespace rbest
{

  template<typename VECS_TYPE, int STATE_DIM, int OBSERVATION_DIM>
  class ObservationModel
  {
  public:
    using StateVector       = Eigen::Matrix<VECS_TYPE, STATE_DIM, 1>;
    using ObservationVector = Eigen::Matrix<VECS_TYPE, OBSERVATION_DIM, 1>;

  public:
    virtual ObservationVector observe(StateVector const& state) const = 0;
  };
  
}
