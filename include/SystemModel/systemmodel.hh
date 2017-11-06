#pragma once

#include <Eigen/Core>

namespace rbest
{

  /** Base for modelling a system
   *
   * @tparam VECS_TYPE Element type of all vectors being used
   * @tparam STATE_DIM State vector dimensionality
   * @tparam CONTROL_DIM Control vector dimensionality
   */
  template<typename VECS_TYPE, int STATE_DIM, int CONTROL_DIM>
  class SystemModel
  {
  public:
    using StateVector = Eigen::Matrix<VECS_TYPE, STATE_DIM, 1>;
    using ControlVector = Eigen::Matrix<VECS_TYPE, CONTROL_DIM, 1>;
    
  public:
    /** Predict next state given previous state and control vector
     *
     * @returns a noise-free prediction
     */
    virtual StateVector predict(StateVector const& state, ControlVector const& control) = 0;
  };

}
