#pragma once

#include "Filter/filter.hh"
#include "SystemModel/linearsystemmodel.hh"
#include "ObservationModel/linearobservationmodel.hh"

#include <Eigen/Dense>

namespace rbest
{

  template<typename VECS_TYPE, int STATE_DIM, int CONTROL_DIM, int OBSERVATION_DIM>
  class KalmanFilter : public Filter<
    LinearSystemModel<VECS_TYPE, STATE_DIM, CONTROL_DIM>,
    LinearObservationModel<VECS_TYPE, STATE_DIM, CONTROL_DIM>>
  {
  public:
    using Base = Filter<
    LinearSystemModel<VECS_TYPE, STATE_DIM, CONTROL_DIM>,
    LinearObservationModel<VECS_TYPE, STATE_DIM, CONTROL_DIM>>;
    
    using typename Base::StateVector;
    using typename Base::ControlVector;
    using typename Base::ObservationVector;

    using typename Base::SystemModelType;
    using typename Base::ObservationModelType;

    using StateCovar = Eigen::Matrix<VECS_TYPE, STATE_DIM, STATE_DIM>;
    
  public:
    void predict(SystemModelType const& systemModel, ControlVector const& control) override;

    void update(ObservationModelType const& observationModel, ObservationVector const& observation) override;

    void init(StateVector state) override;
    void init(StateVector state, StateCovar covar);
    
    StateVector getState() const override;
    StateCovar getStateCovar() const;
    
  private:
    StateVector d_state;
    StateCovar  d_stateCovar;
  };


  template<typename VECS_TYPE, int STATE_DIM, int CONTROL_DIM, int OBSERVATION_DIM>
  void KalmanFilter<VECS_TYPE, STATE_DIM, CONTROL_DIM, OBSERVATION_DIM>::predict(SystemModelType const& systemModel, ControlVector const& control)
  {
  }
  
  template<typename VECS_TYPE, int STATE_DIM, int CONTROL_DIM, int OBSERVATION_DIM>
  void KalmanFilter<VECS_TYPE, STATE_DIM, CONTROL_DIM, OBSERVATION_DIM>::update(ObservationModelType const& observationModel, ObservationVector const& observation)
  {
    // O x S
    Eigen::Matrix<VECS_TYPE, OBSERVATION_DIM, STATE_DIM> const& obsMat =
      observationModel.getObservationMatrix();

    // O x 1
    ObservationVector residual = observation - obsMat * d_state;

    // O x O
    Eigen::Matrix<VECS_TYPE, OBSERVATION_DIM, OBSERVATION_DIM> residualCovar =
      observationModel.getObservationNoiseCovar() + obsMat * d_stateCovar * obsMat.transpose();

    // S x O
    Eigen::Matrix<VECS_TYPE, STATE_DIM, OBSERVATION_DIM> gain =
      d_stateCovar * obsMat.transpose() * residualCovar.inverse();

    d_state = d_state + gain * residual;
    d_stateCovar = d_stateCovar - gain * residualCovar * gain.transpose();
  }

  template<typename VECS_TYPE, int STATE_DIM, int CONTROL_DIM, int OBSERVATION_DIM>
  void KalmanFilter<VECS_TYPE, STATE_DIM, CONTROL_DIM, OBSERVATION_DIM>::init(StateVector state)
  {
    d_state = std::move(state);
    d_stateCovar = StateCovar::Identity();
  }
  
  template<typename VECS_TYPE, int STATE_DIM, int CONTROL_DIM, int OBSERVATION_DIM>
  void KalmanFilter<VECS_TYPE, STATE_DIM, CONTROL_DIM, OBSERVATION_DIM>::init(StateVector state, StateCovar covar)
  {
    d_state = std::move(state);
    d_stateCovar = std::move(covar);
  }
  
  template<typename VECS_TYPE, int STATE_DIM, int CONTROL_DIM, int OBSERVATION_DIM>
  typename KalmanFilter<VECS_TYPE, STATE_DIM, CONTROL_DIM, OBSERVATION_DIM>::StateVector KalmanFilter<VECS_TYPE, STATE_DIM, CONTROL_DIM, OBSERVATION_DIM>::getState() const
  {
    return d_state;
  }

  template<typename VECS_TYPE, int STATE_DIM, int CONTROL_DIM, int OBSERVATION_DIM>
  typename KalmanFilter<VECS_TYPE, STATE_DIM, CONTROL_DIM, OBSERVATION_DIM>::StateCovar KalmanFilter<VECS_TYPE, STATE_DIM, CONTROL_DIM, OBSERVATION_DIM>::getStateCovar() const
  {
    return d_stateCovar;
  }
}
