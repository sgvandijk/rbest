#pragma once

#include "ObservationModel/differentiableobservationmodel.hh"

namespace rbest
{

  /** Model of observations that are linear functions of state with Gaussian noise
   *
   * Models observations as:
   *
   *     o = H * x + v
   *
   * , where H is the observation matrix and v is the (Gaussian)
   * observation noise.
   */
  template<typename VECS_TYPE, int STATE_DIM, int OBSERVATION_DIM>
  class LinearObservationModel : public DifferentiableObservationModel<VECS_TYPE, STATE_DIM, OBSERVATION_DIM>
  {
  public:
    using Base = DifferentiableObservationModel<VECS_TYPE, STATE_DIM, OBSERVATION_DIM>;
    using typename Base::StateVector;
    using typename Base::ObservationVector;
    using typename Base::Jacobian;
    
    using ObservationMatrix = Eigen::Matrix<VECS_TYPE, OBSERVATION_DIM, STATE_DIM>;

  public:
    void setObservationMatrix(ObservationMatrix mat);

    ObservationMatrix const& getObservationMatrix() const;

    Jacobian getJacobian(StateVector const& state) const override;
    
    ObservationVector observe(StateVector const& state) const override;
    
  private:
    ObservationMatrix d_observationMatrix;
  };

  
  template<typename VECS_TYPE, int STATE_DIM, int OBSERVATION_DIM>
  void LinearObservationModel<VECS_TYPE, STATE_DIM, OBSERVATION_DIM>::setObservationMatrix(ObservationMatrix mat)
  {
    d_observationMatrix = mat;
  }

  template<typename VECS_TYPE, int STATE_DIM, int OBSERVATION_DIM>
  typename LinearObservationModel<VECS_TYPE, STATE_DIM, OBSERVATION_DIM>::ObservationMatrix const& LinearObservationModel<VECS_TYPE, STATE_DIM, OBSERVATION_DIM>::getObservationMatrix() const
  {
    return d_observationMatrix;
  }

  template<typename VECS_TYPE, int STATE_DIM, int OBSERVATION_DIM>
  typename LinearObservationModel<VECS_TYPE, STATE_DIM, OBSERVATION_DIM>::Jacobian LinearObservationModel<VECS_TYPE, STATE_DIM, OBSERVATION_DIM>::getJacobian(StateVector const& state) const
  {
    return d_observationMatrix;
  }

  template<typename VECS_TYPE, int STATE_DIM, int OBSERVATION_DIM>
  typename LinearObservationModel<VECS_TYPE, STATE_DIM, OBSERVATION_DIM>::ObservationVector LinearObservationModel<VECS_TYPE, STATE_DIM, OBSERVATION_DIM>::observe(StateVector const& state) const
  {
    return d_observationMatrix * state;
  }
}
