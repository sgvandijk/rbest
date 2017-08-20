#pragma once

#include "Filter/filter.hh"
#include "SystemModel/GaussianSystemModel/gaussiansystemmodel.hh"
#include "ObservationModel/GaussianObservationModel/gaussianobservationmodel.hh"

namespace rbest
{

  template<typename VECS_TYPE, int STATE_DIM, int CONTROL_DIM, int OBSERVATION_DIM>
  class KalmanFilter : public Filter<
    GaussianSystemModel<VECS_TYPE, STATE_DIM, CONTROL_DIM>,
    GaussianObservationModel<VECS_TYPE, STATE_DIM, CONTROL_DIM>>
  {
  public:
    using Base = Filter<
    GaussianSystemModel<VECS_TYPE, STATE_DIM, CONTROL_DIM>,
    GaussianObservationModel<VECS_TYPE, STATE_DIM, CONTROL_DIM>>;
    
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
    
    StateVector getState() const override;
    
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
    d_state = observationModel
  }

  template<typename VECS_TYPE, int STATE_DIM, int CONTROL_DIM, int OBSERVATION_DIM>
  void KalmanFilter<VECS_TYPE, STATE_DIM, CONTROL_DIM, OBSERVATION_DIM>::init(StateVector state)
  {
    d_state = std::move(state);
    d_stateCovar = StateCovar::Identity();
  }
  
  template<typename VECS_TYPE, int STATE_DIM, int CONTROL_DIM, int OBSERVATION_DIM>
  typename KalmanFilter<VECS_TYPE, STATE_DIM, CONTROL_DIM, OBSERVATION_DIM>::StateVector KalmanFilter<VECS_TYPE, STATE_DIM, CONTROL_DIM, OBSERVATION_DIM>::getState() const
  {
    return d_state;
  }
}
