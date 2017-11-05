#pragma once

#include "SystemModel/systemmodel.hh"

namespace rbest
{

  template<typename VECS_TYPE, int STATE_DIM, int CONTROL_DIM>
  class GaussianSystemModel : public SystemModel<VECS_TYPE, STATE_DIM, CONTROL_DIM>
  {
  public:
    using SystemNoiseCovar = Eigen::Matrix<VECS_TYPE, STATE_DIM, STATE_DIM>;
    
  public:
    void setSystemNoiseCovar(SystemNoiseCovar covar);

    SystemNoiseCovar const& getSystemNoiseCovar() const;

  private:
    SystemNoiseCovar d_systemNoiseCovar;
  };


  template<typename VECS_TYPE, int STATE_DIM, int CONTROL_DIM>
  void GaussianSystemModel<VECS_TYPE, STATE_DIM, CONTROL_DIM>::setSystemNoiseCovar(SystemNoiseCovar covar)
  {
    d_systemNoiseCovar = std::move(covar);
  }

  template<typename VECS_TYPE, int STATE_DIM, int CONTROL_DIM>
  typename GaussianSystemModel<VECS_TYPE, STATE_DIM, CONTROL_DIM>::SystemNoiseCovar const& GaussianSystemModel<VECS_TYPE, STATE_DIM, CONTROL_DIM>::getSystemNoiseCovar() const
  {
    return d_systemNoiseCovar;
  }

}
