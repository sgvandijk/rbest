#pragma once

#include "SystemModel/gaussiansystemmodel.hh"

namespace rbest
{

  /** Model of a nonlinear differentiable system with gaussian noise
   *
   * Models the evolution of the system as:
   *
   *     x1 = f(x0, u1) + w 
   *
   * , where f is the differentiable transisition function and w is
   * the (Gaussian) process noise.
   */
  template<typename VECS_TYPE, int STATE_DIM, int CONTROL_DIM>
  class DifferentiableSystemModel : public GaussianSystemModel<VECS_TYPE, STATE_DIM, CONTROL_DIM>
  {
  public:
    using Base = SystemModel<VECS_TYPE, STATE_DIM, CONTROL_DIM>;
    using typename Base::StateVector;
    using typename Base::ControlVector;

    using Jacobian = Eigen::Matrix<VECS_TYPE, STATE_DIM, STATE_DIM>;

    /** Determine system Jacobian
     *
     * Should provide the matrix of all first-order partial
     * derivatives of the transition function f(x0, u1) at given state
     * x0 and control u1.
     */
    virtual Jacobian getJacobian(StateVector const& state, ControlVector const& control) = 0;
  };
}
