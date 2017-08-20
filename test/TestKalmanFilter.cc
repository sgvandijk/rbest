#include <gtest/gtest.h>

#include "Filter/KalmanFilter/kalmanfilter.hh"

class DummySystemModel : public rbest::SystemModel<double, 2, 2>
{
public:
  
  StateVector predict(StateVector const& state, ControlVector const& control) override
  {
    return state + control;
  }
};

TEST(TestKalmanFilter, predict)
{
  auto filter = rbest::KalmanFilter<double, 2, 2, 2>{};
  filter.init(Eigen::Vector2d::Zero());
  
  ASSERT_EQ(Eigen::Vector2d::Zero(), filter.getState());

  filter.predict(DummySystemModel{}, Eigen::Vector2d::Zero());

  ASSERT_EQ(Eigen::Vector2d::Zero(), filter.getState());
}
