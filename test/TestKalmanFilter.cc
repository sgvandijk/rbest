#include <gtest/gtest.h>

#include "Filter/KalmanFilter/kalmanfilter.hh"

#include <algorithm>
#include <array>

class DummySystemModel : public rbest::GaussianSystemModel<double, 2, 2>
{
public:
  
  StateVector predict(StateVector const& state, ControlVector const& control) override
  {
    return state + control;
  }
};

TEST(TestKalmanFilter, predict_zero)
{
  auto filter = rbest::KalmanFilter<double, 2, 2, 2>{};
  filter.init(Eigen::Vector2d::Zero());
  
  ASSERT_EQ(Eigen::Vector2d::Zero(), filter.getState());

  filter.predict(DummySystemModel{}, Eigen::Vector2d::Zero());

  ASSERT_EQ(Eigen::Vector2d::Zero(), filter.getState());
}


class StationarySystemModel : public rbest::GaussianSystemModel<double, 1, 1>
{
public:
  StateVector predict(StateVector const& state, ControlVector const& control) override
  {
    return state;
  }
};

TEST(TestKalmanFilter, constant_filter)
{
  // Numerical example from:
  // http://bilgin.esme.org/BitsAndBytes/KalmanFilterforDummies
  
  using FilterType = rbest::KalmanFilter<double, 1, 1, 1>;
  using ObsModelType = rbest::LinearObservationModel<double, 1, 1>;
  
  auto filter = FilterType{};
  filter.init(FilterType::StateVector{0.});

  auto systemModel = StationarySystemModel{};
  auto observationModel = ObsModelType{};
  observationModel.setObservationMatrix(ObsModelType::ObservationMatrix::Identity());
  observationModel.setObservationNoiseCovar(ObsModelType::ObservationNoiseCovar::Identity() * 0.1);
  
  auto observations = std::array<double, 10>{{
      0.39,
      0.50,
      0.48,
      0.29,
      0.25,
      0.32,
      0.34,
      0.48,
      0.41,
      0.45
    }};


  auto states = std::array<std::pair<double, double>, 10>{};
  
  std::transform(observations.begin(), observations.end(), states.begin(), [&](double obs) {
      filter.predict(systemModel, FilterType::ControlVector{0.0});
      filter.update(observationModel, FilterType::ObservationVector{obs});
      return std::make_pair(filter.getState()[0], filter.getStateCovar()(0, 0));
    });
  
  auto expectedStates = std::array<double, 10>{{
      0.355,
      0.424,
      0.442,
      0.405,
      0.375,
      0.365,
      0.362,
      0.377,
      0.380,
      0.387
    }};

  auto expectedCovars = std::array<double, 10>{{
      0.091,
      0.048,
      0.032,
      0.024,
      0.020,
      0.016,
      0.014,
      0.012,
      0.011,
      0.010
    }};

  for (unsigned i = 0; i < states.size(); ++i)
  {
    ASSERT_NEAR(expectedStates[i], states[i].first, 0.001);
    ASSERT_NEAR(expectedCovars[i], states[i].second, 0.001);
  }
}
