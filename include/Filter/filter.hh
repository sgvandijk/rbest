#pragma once

#include <Eigen/Core>

#include "SystemModel/systemmodel.hh"
#include "ObservationModel/observationmodel.hh"

namespace rbest
{

  /** Base of a recursive Bayesian filter
   *
   * A recursive Bayesian filter estimates the probability
   * distribution of the state of a system by recursively applying two
   * steps: predict and update.
   * 
   * In the prediction step the state distribution estimate is updated
   * according to some system model and the control applied to the
   * system in the corresponding timestep.
   *
   * In the update step the state distribution estimate is refined
   * according to some observation model and the observation received
   * in the corresponsing timestep.
   *
   * Specific filter implementations are made by extending this class
   * and defining the internal distribution representation, and the
   * type of system and observation models they accept.
   *
   * @tparam SYSTEM_MODEL_TYPE The class of system models the filter
   * accepts. Should be a subclass of SystemModel.
   * @tparam OBSERVATION_MODEL_TYPE The class of observation models
   * the filter accepts. Should be a subclass of ObservationModel.
   */
  template<typename SYSTEM_MODEL_TYPE, typename OBSERVATION_MODEL_TYPE>
  class Filter
  {
  public:
    using StateVector       = typename SYSTEM_MODEL_TYPE::StateVector;
    using ControlVector     = typename SYSTEM_MODEL_TYPE::ControlVector;
    using ObservationVector = typename OBSERVATION_MODEL_TYPE::ObservationVector;

    using SystemModelType      = SYSTEM_MODEL_TYPE;
    using ObservationModelType = OBSERVATION_MODEL_TYPE;
    
  public:
    /** Apply system model and control
     *
     * Updates state distribution estimate based on system model and control vector.
     */
    virtual void predict(SystemModelType const& systemModel, ControlVector const& control) = 0;

    /** Refine given observation
     *
     * Updates state distribution estimate based on observation model and observation vector.
     */
    virtual void update(ObservationModelType const& observationModel, ObservationVector const& observation) = 0;

    /** Reset filter with new start state
     */
    virtual void init(StateVector state) = 0;

    /** Get the current state
     *
     * @returns the most likely current state
     */
    virtual StateVector getState() const = 0;
  };
}
