#include <gtest/gtest.h>

#include "ObservationModel/observationmodel.hh"
#include "ObservationModel/linearobservationmodel.hh"

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
