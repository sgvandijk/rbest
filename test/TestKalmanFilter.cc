#include <gtest/gtest.h>

#include "Filter/kalmanfilter.hh"

#include <algorithm>
#include <array>

class DummySystemModel : public rbest::LinearSystemModel<double, 2, 2>
{
public:
  
  StateVector predict(StateVector const& state, ControlVector const& control) const override
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


TEST(TestKalmanFilter, constant_filter)
{
  // Numerical example from:
  // http://bilgin.esme.org/BitsAndBytes/KalmanFilterforDummies
  
  using FilterType = rbest::KalmanFilter<double, 1, 1, 1>;
  using ObsModelType = FilterType::ObservationModelType;
  using SystemModelType = FilterType::SystemModelType;
  
  auto filter = FilterType{};
  filter.init(FilterType::StateVector{0.});

  auto systemModel = SystemModelType{};
  systemModel.setTransitionMatrix(SystemModelType::TransitionMatrix::Identity());
  systemModel.setSystemNoiseCovar(SystemModelType::SystemNoiseCovar::Zero());
  
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

TEST(KalmanFilter, tank_filling_filter)
{
  // Example from:
  // https://www.cs.cornell.edu/courses/cs4758/2012sp/materials/MI63slides.pdf
  // Numbers from:
  // https://www.cs.auckland.ac.nz/~rklette/TeachAuckland.html/AdvancedMI/KalmanExample.xls
  
  using FilterType = rbest::KalmanFilter<double, 2, 1, 1>;
  using ObsModelType = FilterType::ObservationModelType;
  using SystemModelType = FilterType::SystemModelType;

  auto filter = FilterType{};
  filter.init(FilterType::StateVector{0., 0.},
              (FilterType::StateCovar{} << 1000., 0., 0., 1000.).finished());

  auto systemModel = SystemModelType{};
  systemModel.setTransitionMatrix((SystemModelType::TransitionMatrix{} << 1., 1., 0., 1.).finished());
  auto qf = 1e-5;
  systemModel.setSystemNoiseCovar((SystemModelType::SystemNoiseCovar{} << qf / 3, qf / 2, qf / 2, qf).finished());

  auto observationModel = ObsModelType{};
  observationModel.setObservationMatrix((ObsModelType::ObservationMatrix{} << 1., 0.).finished());
  observationModel.setObservationNoiseCovar((ObsModelType::ObservationNoiseCovar{} << 0.1).finished());

  auto observations = std::array<double, 10>{{
      0.0040,
      0.1589,
      0.2494,
      0.5480,
      0.3580,
      0.6823,
      0.5774,
      0.8580,
      1.0105,
      1.0530
    }};

  auto states = std::array<std::pair<double, double>, 10>{};
  
  std::transform(observations.begin(), observations.end(), states.begin(), [&](double obs) {
      std::cout << "----------\n";
      filter.predict(systemModel, FilterType::ControlVector{0.0});
      std::cout << " PREDICT\n";
      std::cout << filter.getState() << "\n";
      std::cout << filter.getStateCovar() << "\n";
      filter.update(observationModel, FilterType::ObservationVector{obs});
      std::cout << " UPDATE\n";
      std::cout << filter.getState() << "\n";
      std::cout << filter.getStateCovar() << "\n";
      return std::make_pair(filter.getState()[0], filter.getStateCovar()(0, 0));
    });
  
  auto expectedStates = std::array<double, 10>{{
      0.0040,
      0.1589,
      0.2601,
      0.4984,
      0.4831,
      0.6397,
      0.6764,
      0.8120,
      0.9550,
      1.0643
    }};

  auto expectedCovars = std::array<double, 10>{{
      0.1000,
      0.1000,
      0.0833,
      0.0700,
      0.0600,
      0.0524,
      0.0464,
      0.0417,
      0.0378,
      0.0346
    }};
  
  for (unsigned i = 0; i < states.size(); ++i)
  {
    ASSERT_NEAR(expectedStates[i], states[i].first, 0.0005);
    ASSERT_NEAR(expectedCovars[i], states[i].second, 0.0005);
  }
}
