#pragma once

#include "SystemModel/systemmodel.hh"

namespace rbest
{

  template<typename VECS_TYPE, int STATE_DIM, int CONTROL_DIM>
  class LinearSystemModel : public SystemModel<VECS_TYPE, STATE_DIM, CONTROL_DIM>
  {
  public:
    using Base = SystemModel<VECS_TYPE, STATE_DIM, CONTROL_DIM>;
    using typename Base::StateVector;
    using typename Base::ControlVector;
    
    using TransitionMatrix = Eigen::Matrix<VECS_TYPE, STATE_DIM, STATE_DIM>;
    using ControlMatrix = Eigen::Matrix<VECS_TYPE, STATE_DIM, CONTROL_DIM>;

  public:
    void setTransitionMatrix(TransitionMatrix mat);

    TransitionMatrix const& getTransitionMatrix() const;

    void setControlMatrix(ControlMatrix mat);

    ControlMatrix const& getControlMatrix() const;

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
  typename LinearSystemModel<VECS_TYPE, STATE_DIM, CONTROL_DIM>::StateVector LinearSystemModel<VECS_TYPE, STATE_DIM, CONTROL_DIM>::predict(StateVector const& state, ControlVector const& control)
  {
    return d_transitionMatrix * state + d_controlMatrix * control;
  }
}
