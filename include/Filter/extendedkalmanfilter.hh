#pragma once

#include "Filter/filter.hh"
#include "SystemModel/differentiablesystemmodel.hh"
#include "ObservationModel/differentiableobservationmodel.hh"

#include <Eigen/Dense>

namespace rbest
{

  template<typename SYSTEM_MODEL_TYPE, typename OBSERVATION_MODEL_TYPE>
  class ExtendedKalmanFilter : public Filter<SYSTEM_MODEL_TYPE, OBSERVATION_MODEL_TYPE>
  {
  public:
    using Base = Filter<SYSTEM_MODEL_TYPE, OBSERVATION_MODEL_TYPE>;

    using typename Base::StateVector;
    using typename Base::ControlVector;
    using typename Base::ObservationVector;

    using typename Base::SystemModelType;
    using typename Base::ObservationModelType;

    using VECS_TYPE = typename StateVector::value_type;
    static constexpr int STATE_DIM = StateVector::RowsAtCompileTime;
    static constexpr int CONTROL_DIM = ControlVector::RowsAtCompileTime;
    static constexpr int OBSERVATION_DIM = ObservationVector::RowsAtCompileTime;

    using StateCovar = Eigen::Matrix<VECS_TYPE, STATE_DIM, STATE_DIM>;
    
    static_assert(std::is_base_of<
                  DifferentiableSystemModel<VECS_TYPE, STATE_DIM, CONTROL_DIM>,
                  SYSTEM_MODEL_TYPE>::value, "System model must be differentiable");

    static_assert(std::is_base_of<
                  DifferentiableObservationModel<VECS_TYPE, STATE_DIM, OBSERVATION_DIM>,
                  OBSERVATION_MODEL_TYPE>::value, "Observation model must be differentiable");

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

  template<typename SYSTEM_MODEL_TYPE, typename OBSERVATION_MODEL_TYPE>
  void ExtendedKalmanFilter<SYSTEM_MODEL_TYPE, OBSERVATION_MODEL_TYPE>::predict(SystemModelType const& systemModel, ControlVector const& control)
  {
    auto transMat = systemModel.getJacobian(d_state, control);
    
    d_state = systemModel.predict(d_state, control);
    d_stateCovar = transMat * d_stateCovar * transMat.transpose() + systemModel.getSystemNoiseCovar();
  }

  
  template<typename SYSTEM_MODEL_TYPE, typename OBSERVATION_MODEL_TYPE>
  void ExtendedKalmanFilter<SYSTEM_MODEL_TYPE, OBSERVATION_MODEL_TYPE>::update(ObservationModelType const& observationModel, ObservationVector const& observation)
  {
    auto obsMat = observationModel.getJacobian(d_state);
    
    ObservationVector residual = observation - observationModel.observe(d_state);
    Eigen::Matrix<VECS_TYPE, OBSERVATION_DIM, OBSERVATION_DIM> residualCovar =
      observationModel.getObservationNoiseCovar() + obsMat * d_stateCovar * obsMat.transpose();
    Eigen::Matrix<VECS_TYPE, STATE_DIM, OBSERVATION_DIM> gain =
      d_stateCovar * obsMat.transpose() * residualCovar.inverse();

    d_state = d_state + gain * residual;
    d_stateCovar = d_stateCovar - gain * residualCovar * gain.transpose();
  }

  template<typename SYSTEM_MODEL_TYPE, typename OBSERVATION_MODEL_TYPE>
  void ExtendedKalmanFilter<SYSTEM_MODEL_TYPE, OBSERVATION_MODEL_TYPE>::init(StateVector state)
  {
    d_state = std::move(state);
    d_stateCovar = StateCovar::Identity();
  }
  
  template<typename SYSTEM_MODEL_TYPE, typename OBSERVATION_MODEL_TYPE>
  void ExtendedKalmanFilter<SYSTEM_MODEL_TYPE, OBSERVATION_MODEL_TYPE>::init(StateVector state, StateCovar covar)
  {
    d_state = std::move(state);
    d_stateCovar = std::move(covar);
  }
  
  template<typename SYSTEM_MODEL_TYPE, typename OBSERVATION_MODEL_TYPE>
  typename ExtendedKalmanFilter<SYSTEM_MODEL_TYPE, OBSERVATION_MODEL_TYPE>::StateVector ExtendedKalmanFilter<SYSTEM_MODEL_TYPE, OBSERVATION_MODEL_TYPE>::getState() const
  {
    return d_state;
  }

  template<typename SYSTEM_MODEL_TYPE, typename OBSERVATION_MODEL_TYPE>
  typename ExtendedKalmanFilter<SYSTEM_MODEL_TYPE, OBSERVATION_MODEL_TYPE>::StateCovar ExtendedKalmanFilter<SYSTEM_MODEL_TYPE, OBSERVATION_MODEL_TYPE>::getStateCovar() const
  {
    return d_stateCovar;
  }

}
