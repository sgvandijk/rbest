#pragma once

#include "SystemModel/differentiablesystemmodel.hh"

namespace rbest
{

  /** Model of a linear system with gaussian noise
   *
   * Models the evolution of the system as:
   *
   *     x1 = F * x0 + B * u1 + w
   *
   * , where F is the transition matrix, B is the control matrix, u1
   * is the control vector and w is the (Gaussian) process noise.
   */
  template<typename VECS_TYPE, int STATE_DIM, int CONTROL_DIM>
  class LinearSystemModel : public DifferentiableSystemModel<VECS_TYPE, STATE_DIM, CONTROL_DIM>
  {
  public:
    using Base = DifferentiableSystemModel<VECS_TYPE, STATE_DIM, CONTROL_DIM>;
    using typename Base::StateVector;
    using typename Base::ControlVector;
    using typename Base::Jacobian;
    
    using TransitionMatrix = Eigen::Matrix<VECS_TYPE, STATE_DIM, STATE_DIM>;
    using ControlMatrix = Eigen::Matrix<VECS_TYPE, STATE_DIM, CONTROL_DIM>;
  public:
    void setTransitionMatrix(TransitionMatrix mat);

    TransitionMatrix const& getTransitionMatrix() const;

    void setControlMatrix(ControlMatrix mat);

    ControlMatrix const& getControlMatrix() const;

    Jacobian getJacobian(StateVector const& state, ControlVector const& control) const override;
    
    StateVector predict(StateVector const& state, ControlVector const& control) override;
    
  private:
    TransitionMatrix d_transitionMatrix;
    ControlMatrix d_controlMatrix;
  };


  template<typename VECS_TYPE, int STATE_DIM, int CONTROL_DIM>
  void LinearSystemModel<VECS_TYPE, STATE_DIM, CONTROL_DIM>::setTransitionMatrix(TransitionMatrix mat)
  {
    d_transitionMatrix = std::move(mat);
  }

  template<typename VECS_TYPE, int STATE_DIM, int CONTROL_DIM>
  typename LinearSystemModel<VECS_TYPE, STATE_DIM, CONTROL_DIM>::TransitionMatrix const& LinearSystemModel<VECS_TYPE, STATE_DIM, CONTROL_DIM>::getTransitionMatrix() const
  {
    return d_transitionMatrix;
  }

  template<typename VECS_TYPE, int STATE_DIM, int CONTROL_DIM>
  void LinearSystemModel<VECS_TYPE, STATE_DIM, CONTROL_DIM>::setControlMatrix(ControlMatrix mat)
  {
    d_controlMatrix = std::move(mat);
  }

  template<typename VECS_TYPE, int STATE_DIM, int CONTROL_DIM>
  typename LinearSystemModel<VECS_TYPE, STATE_DIM, CONTROL_DIM>::ControlMatrix const& LinearSystemModel<VECS_TYPE, STATE_DIM, CONTROL_DIM>::getControlMatrix() const
  {
    return d_controlMatrix;
  }

  template<typename VECS_TYPE, int STATE_DIM, int CONTROL_DIM>
  typename LinearSystemModel<VECS_TYPE, STATE_DIM, CONTROL_DIM>::Jacobian LinearSystemModel<VECS_TYPE, STATE_DIM, CONTROL_DIM>::getJacobian(StateVector const& state, ControlVector const& control) const
  {
    return d_transitionMatrix;
  }

  template<typename VECS_TYPE, int STATE_DIM, int CONTROL_DIM>
  typename LinearSystemModel<VECS_TYPE, STATE_DIM, CONTROL_DIM>::StateVector LinearSystemModel<VECS_TYPE, STATE_DIM, CONTROL_DIM>::predict(StateVector const& state, ControlVector const& control)
  {
    return d_transitionMatrix * state + d_controlMatrix * control;
  }
}
