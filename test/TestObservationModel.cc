#include <gtest/gtest.h>

#include "ObservationModel/observationmodel.hh"
#include "ObservationModel/linearobservationmodel.hh"
#include "ObservationModel/differrentiableobservationmodel.hh"

#include <cmath>

class DummyObservationModel : public rbest::ObservationModel<double, 2, 2>
{
public:
  using Base = rbest::ObservationModel<double, 2, 2>;
  using Base::StateVector;
  using Base::ObservationVector;

  ObservationVector observe(StateVector const& state) const override
  {
    return state;
  }
};

TEST(TestObservationModel, inheritance)
{
  auto model = DummyObservationModel{};
  ASSERT_EQ(Eigen::Vector2d(1., 1.), model.observe(Eigen::Vector2d::Ones()));
}

TEST(TestObservationModel, linear)
{
  using ModelType = rbest::LinearObservationModel<double, 1, 1>;
  auto model = ModelType{};
  model.setObservationMatrix((ModelType::ObservationMatrix{} << 2.0).finished());

  ASSERT_EQ(ModelType::ObservationVector{4.0}, model.observe(ModelType::StateVector{2.0}));
}

class LogarithmicObservationModel : public rbest::DifferrentiableObservationModel<double, 1, 1>
{
public:
  using Base = rbest::DifferrentiableObservationModel<double, 1, 1>;
  using StateVector = typename Base::StateVector;
  using ObservationVector = typename Base::ObservationVector;
  using Jacobian = typename Base::Jacobian;

public:

  Jacobian getJacobian(StateVector const& state) const override
  {
    return 1 / state.array();
  }

  ObservationVector observe(StateVector const& state) const override
  {
    return state.array().log();
  }
};

TEST(TestObservationModel, differentiable)
{
  auto model = LogarithmicObservationModel{};

  for (int i = 1; i < 100; ++i)
    ASSERT_NEAR(std::log(i), model.observe(LogarithmicObservationModel::StateVector{i})[0], 1e-6);
}
