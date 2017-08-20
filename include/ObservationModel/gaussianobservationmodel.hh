#pragma once

#include "ObservationModel/observationmodel.hh"

namespace rbest
{
  /** Model for observations with gaussian noise
   */
  template<typename VECS_TYPE, int STATE_DIM, int OBSERVATION_DIM>
  class GaussianObservationModel : public ObservationModel<VECS_TYPE, STATE_DIM, OBSERVATION_DIM>
  {
  public:
    using ObservationNoiseCovar = Eigen::Matrix<VECS_TYPE, STATE_DIM, STATE_DIM>;

  public:
    void setObservationNoiseCovar(ObservationNoiseCovar covar);

    ObservationNoiseCovar const& getObservationNoiseCovar() const;

  private:
    ObservationNoiseCovar d_observationNoiseCovar;
  };


  template<typename VECS_TYPE, int STATE_DIM, int OBSERVATION_DIM>
  void GaussianObservationModel<VECS_TYPE, STATE_DIM, OBSERVATION_DIM>::setObservationNoiseCovar(ObservationNoiseCovar covar)
  {
    d_observationNoiseCovar = std::move(covar);
  }

  template<typename VECS_TYPE, int STATE_DIM, int OBSERVATION_DIM>
  typename GaussianObservationModel<VECS_TYPE, STATE_DIM, OBSERVATION_DIM>::ObservationNoiseCovar const& GaussianObservationModel<VECS_TYPE, STATE_DIM, OBSERVATION_DIM>::getObservationNoiseCovar() const
  {
    return d_observationNoiseCovar;
  }

}
