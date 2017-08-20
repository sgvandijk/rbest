#include <gtest/gtest.h>

#include "SystemModel/systemmodel.hh"

class DummySystemModel : public rbest::SystemModel<double, 2, 2>
{
public:
  using Base = rbest::SystemModel<double, 2, 2>;
  using Base::StateVector;
  using Base::ControlVector;

  StateVector predict(StateVector const& state, ControlVector const& control)
  {
    return state + control;
  }
};

TEST(TestSystemModel, inheritance)
{
  auto model = DummySystemModel{};
  ASSERT_EQ(Eigen::Vector2d(1., 1.), model.predict(Eigen::Vector2d::Zero(), Eigen::Vector2d::Ones()));
}
