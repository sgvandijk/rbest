#include <gtest/gtest.h>

#include "SystemModel/systemmodel.hh"
#include "SystemModel/linearsystemmodel.hh"
#include "SystemModel/differentiablesystemmodel.hh"

#include <cmath>

class DummySystemModel : public rbest::SystemModel<double, 2, 2>
{
public:
  using Base = rbest::SystemModel<double, 2, 2>;
  using Base::StateVector;
  using Base::ControlVector;

  StateVector predict(StateVector const& state, ControlVector const& control) const override
  {
    return state + control;
  }
};

TEST(TestSystemModel, inheritance)
{
  auto model = DummySystemModel{};
  ASSERT_EQ(Eigen::Vector2d(1., 1.), model.predict(Eigen::Vector2d::Zero(), Eigen::Vector2d::Ones()));
}

TEST(TestSystemModel, linear)
{
  using ModelType = rbest::LinearSystemModel<double, 1, 1>;
  auto model = ModelType{};
  model.setTransitionMatrix((ModelType::TransitionMatrix{} << 1.0).finished());
  model.setControlMatrix((ModelType::ControlMatrix{} << 0.5).finished());
  model.setSystemNoiseCovar((ModelType::SystemNoiseCovar{} << 0.1).finished());

  ASSERT_EQ(ModelType::StateVector{1.5}, model.predict(ModelType::StateVector{1.0},
                                                       ModelType::ControlVector{1.0}));
}

/** Model of logistic map
 *
 * Models a system evolving according to:
 *
 *     x1 = r x0 * (1 - x0) = r (x0 - x0^2)
 */
class LogisticMapModel : public rbest::DifferentiableSystemModel<double, 1, 0>
{
public:
  using Base = rbest::DifferentiableSystemModel<double, 1, 0>;
  using StateVector = typename Base::StateVector;
  using ControlVector = typename Base::ControlVector;
  using Jacobian = typename Base::Jacobian;
  
public:
  LogisticMapModel(double r) : mR{r} {}
  
  Jacobian getJacobian(StateVector const& state, ControlVector const& control) const override
  {
    return (state * -2).array() + mR;
  }

  StateVector predict(StateVector const& state, ControlVector const& control) const override
  {
    return mR * state.array() * (-state.array() + 1);
  }

private:
  double mR;
};

TEST(TestSystemModel, differentiable)
{
  auto model = LogisticMapModel{2.0};

  auto state = LogisticMapModel::StateVector{0.9};

  for (int i = 0; i < 16; ++i)
  {
    state = model.predict(state, LogisticMapModel::ControlVector{});
    // For r = 2 there is an exact solution
    // https://en.wikipedia.org/wiki/Logistic_map
    ASSERT_NEAR(.5 - .5 * std::pow(1. - 2 * 0.9, 1 << (i + 1)), state[0], 1e-6);
  }
  
  ASSERT_NEAR(.5, state[0], 1e-6);

  // After conversion, the Jacobian should be near identity
  ASSERT_NEAR(1.0, model.getJacobian(state, LogisticMapModel::ControlVector{})(0, 0), 1e-6);
}
