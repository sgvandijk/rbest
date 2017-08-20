#pragma once

#include <Eigen/Core>

namespace rbest
{

  template<typename VECS_TYPE, int STATE_DIM, int CONTROL_DIM>
  class SystemModel
  {
  public:
    using StateVector = Eigen::Matrix<VECS_TYPE, STATE_DIM, 1>;
    using ControlVector = Eigen::Matrix<VECS_TYPE, CONTROL_DIM, 1>;
    
  public:
    virtual StateVector predict(StateVector const& state, ControlVector const& control) = 0;
  };

}
