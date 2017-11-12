#pragma once

#include "ObservationModel/gaussianobservationmodel.hh"

namespace rbest
{

  /** Model of a (possibly nonlinear) differentiable observation model with gaussian noise
   *
   * Models observations as:
   *
   *     o = h(x) + v
   *
   * , where h is the differentiable observation function and v is the
   * (Gaussian) observation noise.
   */
  template<typename VECS_TYPE, int STATE_DIM, int OBSERVATION_DIM>
  class DifferentiableObservationModel : public GaussianObservationModel<VECS_TYPE, STATE_DIM, OBSERVATION_DIM>
  {
  public:
    using Base = GaussianObservationModel<VECS_TYPE, STATE_DIM, OBSERVATION_DIM>;
    using StateVector = typename Base::StateVector;
    using ObservationVector = typename Base::ObservationVector;

    using Jacobian = Eigen::Matrix<VECS_TYPE, STATE_DIM, OBSERVATION_DIM>;
    
  public:
    /** Determine observation Jacobian
     *
     * Should provide the matrix of all first-order partial
     * derivatives of the observation function h(x1) at given state
     * x1.
     */
    virtual Jacobian getJacobian(StateVector const& state) const = 0;
  };
}
