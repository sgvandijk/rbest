#pragma once

#include <Eigen/Core>

namespace rbest
{

  /** Base for modelling state observations
   *
   * @tparam VECS_TYPE Element type of all vectors being used
   * @tparam STATE_DIM State vector dimensionality
   * @tparam OBSERVATION_DIM Observation vector dimensionality
   */
  template<typename VECS_TYPE, int STATE_DIM, int OBSERVATION_DIM>
  class ObservationModel
  {
  public:
    using StateVector       = Eigen::Matrix<VECS_TYPE, STATE_DIM, 1>;
    using ObservationVector = Eigen::Matrix<VECS_TYPE, OBSERVATION_DIM, 1>;

  public:
    /** Observe the given state
     *
     * @returns a noise-free observation
     */
    virtual ObservationVector observe(StateVector const& state) const = 0;
  };
  
}
